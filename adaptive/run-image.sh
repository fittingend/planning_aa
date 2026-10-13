#!/usr/bin/env bash
set -euo pipefail
docker run --rm --init --stop-timeout 10 \
  "${IMAGE_TAG:-planning-aa-adaptive:local}" "$@"
