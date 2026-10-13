#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
sdk_dir="${PARA_SDK:-${repo_dir}/../oss-ap/sdk/x86_64}"
sdk_dir="$(cd "${sdk_dir}" && pwd -P)"
test -f "${sdk_dir}/VERSION" || { echo "PARA SDK VERSION file missing: ${sdk_dir}" >&2; exit 1; }
test -f "${sdk_dir}/share/cmake/Modules/PARA/Para.cmake" || {
  echo "PARA CMake module missing: ${sdk_dir}" >&2; exit 1;
}
test "$(uname -m)" = x86_64 || { echo "This bootstrap targets x86_64" >&2; exit 1; }

sdk_version="$(cat "${sdk_dir}/VERSION")"
case "${sdk_version}" in
  R20-11_*_native-linux-gcc) ;;
  *) echo "Unsupported PARA SDK version: ${sdk_version}" >&2; exit 1 ;;
esac
image_tag="${IMAGE_TAG:-planning-aa-adaptive:local}"
docker buildx build --load --platform linux/amd64 \
  --build-context "para_sdk=${sdk_dir}" \
  --build-arg "PARA_SDK_VERSION=${sdk_version}" \
  -f "${repo_dir}/adaptive/Dockerfile" \
  -t "${image_tag}" "${repo_dir}"
echo "Built ${image_tag} with ${sdk_version}"
