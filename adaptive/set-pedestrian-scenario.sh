#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
container="${KATECH_CONTROL_CONTAINER:-planning-aa-control}"

if [[ "$(docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null || true)" != true ]]; then
  echo "Control container is not running: ${container}" >&2
  echo "Start the stack with ./adaptive/run-katech-stack.sh first." >&2
  exit 1
fi

docker exec -i "${container}" bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && python3 -u - "$@"' \
  _ "$@" <"${script_dir}/set-pedestrian-scenario.py"
