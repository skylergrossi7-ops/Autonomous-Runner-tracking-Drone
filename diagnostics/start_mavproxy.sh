#!/usr/bin/env bash
set -eo pipefail

source /home/skyler/venv-ardupilot/bin/activate

exec mavproxy.py \
  --daemon \
  --master=tcp:127.0.0.1:5760 \
  --sitl=127.0.0.1:5501 \
  --out=tcp:127.0.0.1:5770 \
  >>/tmp/mavproxy_current.log 2>&1
