#!/usr/bin/env bash
set -euo pipefail

RDK_PORT="${RDK_PORT:-20501}"
RDK_LIC="${RDK_LIC:-}"

cmd_args=(--platform minimal -SKIPINI -NOUI "-PORT=${RDK_PORT}")

if [[ -n "${RDK_LIC}" ]]; then
  cmd_args+=("-LCMD=${RDK_LIC}")
fi

exec /opt/robodk/bin/RoboDK "${cmd_args[@]}" "$@"
