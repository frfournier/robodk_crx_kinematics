# syntax=docker/dockerfile:1

ARG UBUNTU_IMAGE=ubuntu:24.04
ARG CRX_KINEMATICS_VERSION=0.3.1

FROM ${UBUNTU_IMAGE} AS base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libgl1 \
        libqt5core5a \
        libqt5gui5 \
        libqt5network5 \
        libqt5opengl5 \
        libqt5widgets5 \
        libqt5xml5 \
        libxcb-cursor0 \
        libxcb-keysyms1 \
        libxcb-xinerama0 \
        libxkbcommon-x11-0 \
        libxrender1 \
    && rm -rf /var/lib/apt/lists/*

FROM base AS build

ARG TARGETARCH
ARG ROBODK_ACCEPT_EULA

RUN test "${TARGETARCH}" = "amd64" \
    && test "${ROBODK_ACCEPT_EULA}" = "yes" \
    || (echo >&2 "RoboDK requires linux/amd64 and ROBODK_ACCEPT_EULA=yes"; exit 1)

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        qt5-qmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/robodk-installer
ADD https://cdn.robodk.com/downloads/Install-RoboDK.tar.gz Install-RoboDK.tar.gz

RUN mkdir -p /root/Desktop \
    && tar -xzf Install-RoboDK.tar.gz \
    && chmod +x Install-RoboDK \
    && ./Install-RoboDK \
        --platform minimal \
        --root /opt/robodk \
        --accept-licenses \
        --default-answer \
        --auto-answer OverwriteTargetDirectory=Yes \
        --confirm-command \
        --no-size-checking \
        install \
    && chmod -R a+rX /opt/robodk

WORKDIR /workspace
COPY crxkinematics.pro LICENSE THIRD_PARTY_NOTICES.md ./
COPY include/ include/
COPY src/ src/
COPY third_party/eigen/ third_party/eigen/

WORKDIR /workspace/build
RUN qmake .. "CONFIG+=release" \
    && make -j"$(nproc)" \
    && test -f Release/libcrx_kinematics.so

FROM base AS runtime

ARG CRX_KINEMATICS_VERSION
ARG UID=10001
ARG GID=10001

LABEL org.opencontainers.image.title="RoboDK CRX kinematics" \
      org.opencontainers.image.description="Headless RoboDK API server with CRX custom kinematics" \
      org.opencontainers.image.version="${CRX_KINEMATICS_VERSION}" \
      org.opencontainers.image.licenses="LicenseRef-RoboDK-Proprietary"

COPY --from=build /opt/robodk/ /opt/robodk/
COPY --from=build /workspace/build/Release/ /opt/robodk/bin/robotextensions/
COPY --chmod=0555 .docker/docker-entrypoint.sh .docker/healthcheck.sh /usr/local/bin/

RUN groupadd --gid "${GID}" robodk \
    && useradd \
        --uid "${UID}" \
        --gid "${GID}" \
        --create-home \
        --home-dir /home/robodk \
        --shell /usr/sbin/nologin \
        --no-log-init \
        robodk \
    && install -d -o robodk -g robodk \
        /data \
        /home/robodk/.config/RoboDK

ENV HOME=/home/robodk \
    LANG=C.UTF-8 \
    LD_LIBRARY_PATH=/opt/robodk/bin/lib \
    QT_PLUGIN_PATH=/opt/robodk/bin/plugins \
    QT_QPA_PLATFORM_PLUGIN_PATH=/opt/robodk/bin/plugins \
    QT_QPA_PLATFORM=offscreen \
    XDG_CACHE_HOME=/tmp/robodk-cache \
    XDG_RUNTIME_DIR=/tmp/robodk-runtime

EXPOSE 20501

USER robodk
WORKDIR /data

HEALTHCHECK --interval=30s --timeout=3s --start-period=45s --retries=3 \
    CMD ["/usr/local/bin/healthcheck.sh"]

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
