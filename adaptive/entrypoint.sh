#!/bin/sh
set -eu
export PARA_CORE=/opt/planning-ap
export PARA_CONF="${PARA_CORE}/etc"
export PARA_DATA="${PARA_CORE}/var"
export PARA_APPL="${PARA_CORE}/opt"
export PARA_LOG_LEVEL="${PARA_LOG_LEVEL:-warn}"
export LD_LIBRARY_PATH="${PARA_CORE}/lib:${PARA_CORE}/external/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "${PARA_DATA}"
exec "${PARA_CORE}/bin/EM" "$@"
