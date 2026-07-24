#!/usr/bin/env bash
set -Eeuo pipefail

exec 3<>/dev/tcp/127.0.0.1/20501

# RoboDK API handshake: "RDK_API", followed by three big-endian doubles
# (safe mode, auto-update, skip-status).
printf "RDK_API\n\
\x00\x00\x00\x03\
\x3f\xf0\x00\x00\x00\x00\x00\x00\
\x00\x00\x00\x00\x00\x00\x00\x00\
\x00\x00\x00\x00\x00\x00\x00\x00" >&3

IFS= read -r -t 2 response <&3
[[ "${response}" == "RDK_API" ]]
