#!/usr/bin/env bash
set -Eeuo pipefail

readonly license_file="${RDK_LIC_FILE:-/run/secrets/robodk-license}"
license=""

echo "Starting RoboDK version ${ROBODK_VERSION:-unknown}"

mkdir -p "${XDG_CACHE_HOME}" "${XDG_RUNTIME_DIR}"
chmod 0700 "${XDG_CACHE_HOME}" "${XDG_RUNTIME_DIR}"

if [[ -f "${license_file}" ]]; then
    license="$(<"${license_file}")"
    license="${license//$'\r'/}"
    license="${license//$'\n'/}"
fi

args=(
    --platform minimal
    -SKIPINI
    -SKIPMAINT
    -NOUI
    -PORT=20501
)

if [[ -n "${license}" ]]; then
    args+=("-LCMD=${license}")
fi

exec /opt/robodk/bin/RoboDK "${args[@]}" "$@"
