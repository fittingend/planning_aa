#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source_dir="${KATECH_SOURCE_DIR:-${repo_dir}/../katech-autoware/katech-autosare-new}"
image="${KATECH_CONTROL_IMAGE:-katech-autoware-system-control-arm64:adsw-control}"
base="ghcr.io/oss-test-group/autoware-partition:adsw-control"

test -f "${source_dir}/src/universe/autoware.universe/obigo_launch/launch/sample_adsw_control_run.launch.xml" || {
  echo "KATECH System-in-Control source is missing: ${source_dir}" >&2
  exit 1
}
test "$(docker image inspect --format '{{.Architecture}}' "${base}")" = arm64 || {
  echo "Pull the ARM64 GHCR Control base image first: ${base}" >&2
  exit 1
}

docker buildx build --platform linux/arm64 --load --progress=plain \
  -f "${repo_dir}/adaptive/Dockerfile.katech-control-system" \
  -t "${image}" "${source_dir}"
