#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
arch="${ARCH:-$(uname -m)}"
case "${arch}" in
  x86_64) platform=linux/amd64; docker_arch=amd64; suffix="" ;;
  aarch64) platform=linux/arm64; docker_arch=arm64; suffix=-aarch64 ;;
  *) echo "Unsupported architecture: ${arch}" >&2; exit 1 ;;
esac
bridge_image="${BRIDGE_IMAGE:-decision-ap-bridge:local${suffix}}"
planning_image="${PLANNING_IMAGE:-decision-ap-planning:local${suffix}}"
integrated_image="${INTEGRATED_IMAGE:-decision-ap:local${suffix}}"

for image in "${bridge_image}" "${planning_image}"; do
  docker image inspect "${image}" >/dev/null || {
    echo "Required local image is missing: ${image}" >&2
    exit 1
  }
done
for image in "${bridge_image}" "${planning_image}"; do
  image_arch="$(docker image inspect --format '{{.Architecture}}' "${image}")"
  if [[ "${image_arch}" != "${docker_arch}" ]]; then
    echo "Image architecture mismatch: ${image} is ${image_arch}, expected ${docker_arch}" >&2
    exit 1
  fi
done
bridge_sdk="$(docker image inspect --format '{{index .Config.Labels "para.sdk.version"}}' "${bridge_image}")"
planning_sdk="$(docker image inspect --format '{{index .Config.Labels "para.sdk.version"}}' "${planning_image}")"
if [[ -z "${bridge_sdk}" || "${bridge_sdk}" == "<no value>" || "${bridge_sdk}" == unknown || "${bridge_sdk}" != "${planning_sdk}" ]]; then
  echo "PARA SDK versions must match: bridge=${bridge_sdk}, DecisionAP=${planning_sdk}" >&2
  exit 1
fi

docker buildx build --load --platform "${platform}" \
  --build-arg "BRIDGE_IMAGE=${bridge_image}" \
  --build-arg "PLANNING_IMAGE=${planning_image}" \
  -f "${repo_dir}/adaptive/Dockerfile.integrated" \
  -t "${integrated_image}" "${repo_dir}"
echo "Built ${integrated_image} with ${bridge_sdk}"
