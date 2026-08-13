#!/usr/bin/env bash
set -eo pipefail

root=/mnt/c/Users/skyle/OneDrive/Documents/Drone
workspace=${root}/ros2_ws
export ARDUCOPTER_OVERLAY=${root}/diagnostics/follow_validation.parm
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
source "${workspace}/install/setup.bash"

pids=()
flight_started=0
log_dir=/tmp/follow_while_mapping
mkdir -p "${log_dir}"
result_file="${log_dir}/result.txt"
: > "${result_file}"

start_group() {
  log_file=$1
  shift
  setsid "$@" >"${log_file}" 2>&1 &
  pids+=("$!")
}

land_now() {
  timeout 20 ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'LAND'}" || true
}

cleanup() {
  status=$?
  trap - EXIT INT TERM
  timeout 10 python3 "${workspace}/scripts/set_tracking_enabled.py" false \
    >/dev/null 2>&1 || true
  if [[ "${flight_started}" == "1" ]]; then
    echo "SAFETY: commanding LAND"
    land_now
    sleep 35
  fi
  for pid in "${pids[@]}"; do
    kill -INT -- "-${pid}" 2>/dev/null || true
  done
  sleep 5
  for pid in "${pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  pkill -f '^/home/skyler/ardupilot/build/sitl/bin/arducopter ' \
    2>/dev/null || true
  pkill -x mavros_node 2>/dev/null || true
  pkill -x mavproxy.py 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

if pgrep -x arducopter >/dev/null || pgrep -x mavros_node >/dev/null; then
  echo "Refusing to start: an existing flight stack is running" >&2
  exit 1
fi

wait_for() {
  description=$1
  attempts=$2
  shift 2
  for ((attempt=1; attempt<=attempts; attempt++)); do
    "$@" && { echo "READY: ${description}"; return 0; }
    sleep 2
  done
  echo "FAILED: ${description}" >&2
  return 1
}

port_5760() { ss -ltn | grep -q ':5760 '; }
port_5770() { ss -ltn | grep -q ':5770 '; }
gazebo_camera_ready() {
  python3 "${workspace}/scripts/check_camera_ready.py"
}
connected() {
  timeout 12 ros2 topic echo /mavros/state mavros_msgs/msg/State --once 2>/dev/null \
    | grep -q 'connected: true'
}
target_ready() {
  python3 "${workspace}/scripts/check_target_ready.py"
}
hover_ready() {
  python3 "${workspace}/scripts/check_hover_ready.py"
}

call_service() {
  description=$1 service=$2 type=$3 request=$4
  for attempt in 1 2 3 4 5; do
    response=$(timeout 30 ros2 service call "${service}" "${type}" "${request}") || true
    echo "${response}"
    if [[ "${response}" == *"success=True"* || "${response}" == *"mode_sent=True"* ]]; then
      echo "READY: ${description}"
      return 0
    fi
    sleep 4
  done
  echo "FAILED: ${description}" >&2
  return 1
}

echo "START: Gazebo, perception, AI depth, masking, live TF, and Nav2"
start_group "${log_dir}/integrated_launch.log" \
  ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=true start_rviz:=false use_live_odometry:=true \
  sky_mask_enabled:="${SKY_MASK_ENABLED:-true}"
wait_for "Gazebo RGB camera" 45 gazebo_camera_ready

echo "START: ArduPilot SITL"
start_group "${log_dir}/arducopter.log" \
  bash "${root}/diagnostics/start_arducopter.sh"
wait_for "SITL TCP port" 25 port_5760

echo "START: MAVROS"
start_group "${log_dir}/mavros.log" bash "${root}/diagnostics/start_mavros.sh"
wait_for "MAVROS listener" 20 port_5770

echo "START: MAVProxy"
start_group "${log_dir}/mavproxy.log" bash "${root}/diagnostics/start_mavproxy.sh"
wait_for "MAVROS connection" 45 connected

echo "START: target projection and safe follower"
start_group "${log_dir}/target_vector.log" \
  ros2 run runner_perception target_vector_node --ros-args \
  -p use_sim_time:=true -p runner_timeout_seconds:=8.0 \
  -p camera_upward_pitch_radians:=-0.20
start_group "${log_dir}/runner_follower.log" \
  ros2 run drone_control runner_follower --ros-args \
  --params-file "${workspace}/install/drone_control/share/drone_control/config/follower.yaml" \
  -p use_sim_time:=true -p enabled:=false \
  -p forward_commands_enabled:=true -p publish_to_mavros:=true \
  -p target_timeout_seconds:=8.0 -p obstacle_timeout_seconds:=8.0 \
  -p maximum_forward_speed:=2.5
sleep 25
wait_for "AI runner target before arming" 12 target_ready

call_service "GUIDED mode" /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'GUIDED'}"
sleep 25
call_service "vehicle armed" /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
flight_started=1
call_service "takeoff to 2 m" /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
  "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 2.0}"
sleep 35
wait_for "vehicle above 1 m" 3 hover_ready
wait_for "live AI runner target at hover" 15 target_ready

echo "FOLLOW: enabling motion while monitoring mapping"
python3 "${workspace}/scripts/set_tracking_enabled.py" true
if [[ -n "${FOLLOW_MAPPING_MEDIA_DIR:-}" ]]; then
  mkdir -p "${FOLLOW_MAPPING_MEDIA_DIR}"
  start_group "${log_dir}/media_recorder.log" \
    python3 "${workspace}/scripts/record_dynamic_mapping_demo.py" \
      --output "${FOLLOW_MAPPING_MEDIA_DIR}/follow_with_live_costmap.mp4" \
      --screenshot "${FOLLOW_MAPPING_MEDIA_DIR}/follow_with_live_costmap.png" \
      --duration 50 --fps 5
fi
set +e
python3 "${workspace}/scripts/check_follow_while_mapping.py" \
  2>&1 | tee "${result_file}"
monitor_status=${PIPESTATUS[0]}
set -e

echo "FOLLOW: disabling motion"
python3 "${workspace}/scripts/set_tracking_enabled.py" false
echo "LAND: combined validation complete"
land_now
sleep 40
flight_started=0
if [[ "${monitor_status}" != "0" ]]; then
  echo "VALIDATION_FAILED: monitor reported a failed criterion" >&2
  exit "${monitor_status}"
fi
echo "VALIDATION_COMPLETE"
