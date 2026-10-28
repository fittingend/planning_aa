#!/bin/sh
set -eu
export PARA_CORE=/opt/planning-ap
export PARA_CONF="${PARA_CORE}/etc"
if [ -n "${PLANNING_AA_SOMEIP_ADDRESS:-}" ]; then
  case "${PLANNING_AA_SOMEIP_ADDRESS}:${AP_SOMEIP_ADDRESS:-}" in
    *[!0-9.:]*) echo "Invalid SOME/IP IPv4 address" >&2; exit 1 ;;
  esac
  export PARA_CONF=/run/decision-ap-etc
  cp -a "${PARA_CORE}/etc" "${PARA_CONF}"
  find "${PARA_CONF}" -type f -name '*.json' -exec sed -i \
    -e "s/192\\.168\\.100\\.93/${PLANNING_AA_SOMEIP_ADDRESS}/g" \
    -e "s/192\\.168\\.100\\.92/${AP_SOMEIP_ADDRESS:-10.88.0.92}/g" {} +
fi
export PARA_DATA="${PARA_CORE}/var"
export PARA_APPL="${PARA_CORE}/opt"
export PARA_LOG_LEVEL="${PARA_LOG_LEVEL:-warn}"
export LD_LIBRARY_PATH="${PARA_CORE}/lib:${PARA_CORE}/external/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "${PARA_DATA}"
exec "${PARA_CORE}/bin/EM" "$@"
