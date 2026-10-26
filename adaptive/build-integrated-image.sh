#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
bridge_image="${BRIDGE_IMAGE:-oss-ap-planning-bridge:local}"
planning_image="${PLANNING_IMAGE:-planning-aa-adaptive:local}"
integrated_image="${INTEGRATED_IMAGE:-planning-aa-integrated:local}"

for image in "${bridge_image}" "${planning_image}"; do
  docker image inspect "${image}" >/dev/null || {
    echo "Required local image is missing: ${image}" >&2
    exit 1
  }
done
bridge_sdk="$(docker image inspect --format '{{index .Config.Labels "para.sdk.version"}}' "${bridge_image}")"
planning_sdk="$(docker image inspect --format '{{index .Config.Labels "para.sdk.version"}}' "${planning_image}")"
if [[ -z "${bridge_sdk}" || "${bridge_sdk}" == "<no value>" || "${bridge_sdk}" == unknown || "${bridge_sdk}" != "${planning_sdk}" ]]; then
  echo "PARA SDK versions must match: bridge=${bridge_sdk}, PlanningAA=${planning_sdk}" >&2
  exit 1
fi

docker buildx build --load --platform linux/amd64 \
  --build-arg "BRIDGE_IMAGE=${bridge_image}" \
  --build-arg "PLANNING_IMAGE=${planning_image}" \
  -f "${repo_dir}/adaptive/Dockerfile.integrated" \
  -t "${integrated_image}" "${repo_dir}"
echo "Built ${integrated_image} with ${bridge_sdk}"
