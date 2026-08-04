#!/usr/bin/env bash
set -eo pipefail

set +e
overlay="${ARDUCOPTER_OVERLAY:-/mnt/c/Users/skyle/OneDrive/Documents/Drone/diagnostics/first_test.parm}"
/home/skyler/ardupilot/build/sitl/bin/arducopter \
  --model JSON \
  --speedup 1 \
  --slave 0 \
  --defaults "/home/skyler/ardupilot_gazebo/config/gazebo-iris-gimbal.parm,${overlay}" \
  --sim-address=127.0.0.1 \
  -I0 \
  >>/tmp/arducopter_current.log 2>&1
status=$?
printf 'ArduCopter exit status: %s\n' "$status" >>/tmp/arducopter_current.log
exit "$status"
