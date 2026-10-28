#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
arch="${ARCH:-$(uname -m)}"
case "${arch}" in
  x86_64) platform=linux/amd64; suffix="" ;;
  aarch64) platform=linux/arm64; suffix=-aarch64 ;;
  *) echo "Unsupported architecture: ${arch}" >&2; exit 1 ;;
esac
source_image="${OSS_AP_IMAGE:-decision-ap-bridge:local${suffix}}"
bridge_image="${BRIDGE_IMAGE:-decision-ap-bridge:local${suffix}}"
docker image inspect "${source_image}" >/dev/null || {
  echo "Existing oss-ap runtime image is required: ${source_image}" >&2
  exit 1
}
docker buildx build --load --platform "${platform}" \
  --build-arg "OSS_AP_IMAGE=${source_image}" \
  -f "${repo_dir}/adaptive/Dockerfile.bridge" \
  -t "${bridge_image}" "${repo_dir}"
