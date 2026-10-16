#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source_image="${OSS_AP_IMAGE:-oss-ap-planning-bridge:local}"
bridge_image="${BRIDGE_IMAGE:-oss-ap-planning-bridge:local}"
docker image inspect "${source_image}" >/dev/null || {
  echo "Existing oss-ap runtime image is required: ${source_image}" >&2
  exit 1
}
docker buildx build --load \
  --build-arg "OSS_AP_IMAGE=${source_image}" \
  -f "${repo_dir}/adaptive/Dockerfile.bridge" \
  -t "${bridge_image}" "${repo_dir}"
