#!/usr/bin/env bash
set -euo pipefail

RDK_LIC_FILE="${RDK_LIC_FILE:-/run/secrets/robodk-license}"
RDK_LIC=""

if [[ -f "${RDK_LIC_FILE}" ]]; then
  RDK_LIC="$(tr -d '\r\n' < "${RDK_LIC_FILE}")"
fi

cmd_args=(--platform minimal -SKIPINI -NOUI "-PORT=20501")

if [[ -n "${RDK_LIC}" ]]; then
  cmd_args+=("-LCMD=${RDK_LIC}")
fi

exec /opt/robodk/bin/RoboDK "${cmd_args[@]}" "$@"
