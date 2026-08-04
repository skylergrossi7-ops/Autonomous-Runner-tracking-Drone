#!/usr/bin/env bash
set -eo pipefail

root="/mnt/c/Users/skyle/OneDrive/Documents/Drone"
workspace="${root}/ros2_ws"
result="/tmp/follow_15s_validation.csv"
result_artifact="${root}/diagnostics/follow_15s_validation.csv"
video_result="/tmp/distance_follow_gazebo.mp4"
video_artifact="${root}/diagnostics/distance_follow_gazebo.mp4"
export ARDUCOPTER_OVERLAY="${root}/diagnostics/follow_validation.parm"

source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
export ROS_DOMAIN_ID=0

pids=()
watchdog_pid=""
flight_started=0

start_background() {
  setsid "$@" &
  pids+=("$!")
}

land_now() {
  source /opt/ros/jazzy/setup.bash
  timeout 15 ros2 service call \
    /mavros/set_mode mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'LAND'}" || true
}

cleanup() {
  status=$?
  trap - EXIT INT TERM

  if [[ "${flight_started}" == "1" ]]; then
    printf '%s\n' 'SAFETY: commanding LAND'
    land_now
    sleep 40
  fi

  [[ -z "${watchdog_pid}" ]] || kill "${watchdog_pid}" 2>/dev/null || true
  for pid in "${pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  sleep 3
  for pid in "${pids[@]}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || true
  done
  # Some ROS launchers and the ArduPilot wrapper create child processes that
  # outlive their shell process group. This validator starts from a clean
  # stack, so these exact executable names are in its scope.
  pkill -x arducopter 2>/dev/null || true
  pkill -x mavros_node 2>/dev/null || true
  pkill -x mavproxy.py 2>/dev/null || true
  pkill -x perception_node 2>/dev/null || true
  pkill -x runner_follower 2>/dev/null || true
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

wait_for() {
  description="$1"
  attempts="$2"
  shift 2
  for ((attempt=1; attempt<=attempts; attempt++)); do
    if "$@"; then
      printf 'READY: %s\n' "${description}"
      return 0
    fi
    sleep 2
  done
  printf 'FAILED: %s\n' "${description}" >&2
  return 1
}

port_5760() {
  sockets=$(ss -ltn)
  [[ "${sockets}" == *":5760 "* ]]
}
port_5770() {
  sockets=$(ss -ltn)
  [[ "${sockets}" == *":5770 "* ]]
}
camera_topic() {
  topics=$(gz topic -l)
  [[ $'\n'"${topics}"$'\n' == *$'\n/tracking_camera/image\n'* ]]
}
follower_ready() {
  grep -q 'Runner follower ready' /tmp/follow_15s_follower.log 2>/dev/null
}
recorder_ready() {
  grep -q 'RECORDER_READY' /tmp/follow_15s_recorder.log 2>/dev/null
}
recorder_data_ready() {
  grep -q 'RECORDER_DATA_READY' /tmp/follow_15s_recorder.log 2>/dev/null
}
tracking_enabled() {
  grep -q 'Tracking motion enabled' /tmp/follow_15s_follower.log 2>/dev/null
}
tracking_disabled() {
  grep -q 'Tracking motion disabled' /tmp/follow_15s_follower.log 2>/dev/null
}
video_ready() {
  grep -q 'VIDEO_READY' /tmp/follow_video.log 2>/dev/null
}
connected() {
  # Supplying the type avoids a slow ROS graph type-discovery query while
  # Gazebo, Torch and MAVROS are competing for CPU during startup.
  state=$(timeout 15 ros2 topic echo \
    /mavros/state mavros_msgs/msg/State --once 2>/dev/null) || return 1
  [[ "${state}" == *"connected: true"* ]]
}

call_service_expect() {
  local description="$1"
  local service="$2"
  local type="$3"
  local request="$4"
  local response
  printf 'COMMAND: %s\n' "${description}"
  response=$(timeout 30 ros2 service call "${service}" "${type}" "${request}") || {
    printf 'FAILED: %s service call\n' "${description}" >&2
    return 1
  }
  printf '%s\n' "${response}"
  [[ "${response}" == *"success=True"* || "${response}" == *"mode_sent=True"* ]] || {
    printf 'FAILED: %s rejected\n' "${description}" >&2
    return 1
  }
}

call_service_retry() {
  local description="$1"
  local service="$2"
  local type="$3"
  local request="$4"
  local attempt
  for attempt in 1 2 3 4 5; do
    call_service_expect "${description} (attempt ${attempt})" \
      "${service}" "${type}" "${request}" && return 0
    sleep 4
  done
  printf 'FAILED: %s after five attempts\n' "${description}" >&2
  return 1
}

printf '%s\n' 'START: Gazebo headless'
start_background bash "${root}/diagnostics/start_gazebo_headless.sh"
wait_for 'Gazebo camera topic' 20 camera_topic

printf '%s\n' 'START: camera and lidar bridges'
start_background bash "${root}/diagnostics/start_camera_bridge.sh"
start_background bash "${root}/diagnostics/start_lidar_bridge.sh"
sleep 8
printf '%s\n' 'READY: sensor bridges started'

printf '%s\n' 'START: ArduPilot SITL'
start_background bash "${root}/diagnostics/start_arducopter.sh"
wait_for 'SITL port 5760' 20 port_5760

# MAVROS must listen before MAVProxy connects its TCP output.
printf '%s\n' 'START: MAVROS listener'
start_background bash "${root}/diagnostics/start_mavros.sh"
wait_for 'MAVROS port 5770' 20 port_5770

printf '%s\n' 'START: MAVProxy'
start_background bash "${root}/diagnostics/start_mavproxy.sh"
# MAVROS status subscriptions can stall during the initial full parameter
# transfer. Give the link time to settle; the parameter and mode calls below
# provide direct, response-checked command-path validation.
sleep 20
printf '%s\n' 'READY: MAVLink command path initialization delay complete'

printf '%s\n' 'START: perception only (no follower)'
source /home/skyler/venvs/drone/bin/activate
start_background ros2 run drone_perception perception_node --ros-args \
  --params-file "${workspace}/install/drone_perception/share/drone_perception/config/perception.yaml" \
  -p image_topic:=/tracking_camera/image
sleep 30
printf '%s\n' 'READY: perception initialization delay complete'

printf '%s\n' 'TAKEOFF: simulation-only INS exception loaded at SITL startup'

call_service_retry 'set GUIDED mode' \
  /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
sleep 3
call_service_retry 'arm vehicle' \
  /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
flight_started=1
sleep 3

# Independent watchdog: it will command LAND even if this script stalls.
(
  sleep 600
  printf '%s\n' 'WATCHDOG: 600 seconds elapsed; commanding LAND'
  land_now
) &
watchdog_pid="$!"

call_service_retry 'take off to 2 metres' \
  /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
  "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 2.0}"
sleep 30

printf '%s\n' 'GATE: requiring runner target at hover'
timeout 90 ros2 topic echo \
  /perception/runner_target geometry_msgs/msg/Vector3Stamped --once \
  > /tmp/follow_hover_target.txt
cat /tmp/follow_hover_target.txt

printf '%s\n' 'FOLLOW: enabling commands for exactly 15 seconds'
rm -f "${result}"
rm -f /tmp/follow_15s_recorder.log /tmp/follow_15s_follower.log
python "${root}/diagnostics/record_follow_test.py" "${result}" \
  >/tmp/follow_15s_recorder.log 2>&1 &
recorder_pid="$!"
pids+=("${recorder_pid}")
wait_for 'telemetry recorder initialized' 20 recorder_ready

FOLLOWER_ENABLED=false setsid bash \
  "${root}/diagnostics/start_follower_enabled_only.sh" \
  >/tmp/follow_15s_follower.log 2>&1 &
follower_pid="$!"
pids+=("${follower_pid}")
wait_for 'follower standby initialized' 60 follower_ready
wait_for 'live pose and standby command telemetry' 60 recorder_data_ready

rm -f "${video_result}" /tmp/follow_video.log
python "${root}/diagnostics/record_ros_video.py" \
  /perception/annotated_image "${video_result}" 15 \
  >/tmp/follow_video.log 2>&1 &
video_pid="$!"
pids+=("${video_pid}")
wait_for 'follow video recorder initialized' 30 video_ready

timeout 30 ros2 topic pub --rate 5 --times 5 \
  /tracking/enabled std_msgs/msg/Bool "{data: true}" \
  >/tmp/follow_enable.log 2>&1 &
enable_pid="$!"
wait_for 'follower confirmed motion enabled' 30 tracking_enabled
kill "${enable_pid}" 2>/dev/null || true
printf '%s\n' 'FOLLOW_ACTIVE: 15 second timer started'
sleep 15
timeout 30 ros2 topic pub --rate 5 --times 5 \
  /tracking/enabled std_msgs/msg/Bool "{data: false}" \
  >/tmp/follow_disable.log 2>&1 &
disable_pid="$!"
wait_for 'follower confirmed motion disabled' 30 tracking_disabled
kill "${disable_pid}" 2>/dev/null || true
printf '%s\n' 'FOLLOW_STOPPED: motion disabled after 15 seconds'

wait "${video_pid}"
grep -q 'VIDEO_COMPLETE' /tmp/follow_video.log
cp "${video_result}" "${video_artifact}"
printf 'VIDEO_ARTIFACT: %s\n' "${video_artifact}"

kill -TERM -- "-${follower_pid}" 2>/dev/null || true
wait "${follower_pid}" 2>/dev/null || true

kill -TERM "${recorder_pid}" 2>/dev/null || true
wait "${recorder_pid}" 2>/dev/null || true

rows=$(wc -l < "${result}")
printf 'RESULT_ROWS: %s\n' "${rows}"
python "${root}/diagnostics/analyze_follow_validation.py" "${result}"
cp "${result}" "${result_artifact}"

printf '%s\n' 'FOLLOW_COMPLETE: commanding LAND'
land_now
sleep 40
flight_started=0

printf '%s\n' 'FINAL_STATE'
timeout 8 ros2 topic echo /mavros/state --once || true
timeout 8 gz model -m iris_with_gimbal_lidar -p || true
printf '%s\n' 'VALIDATION_COMPLETE'
