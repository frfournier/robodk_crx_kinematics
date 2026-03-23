# syntax=docker/dockerfile:1
FROM ubuntu:24.04 AS base

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

FROM base AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    curl \
    libgl1 \
    libqt5core5a \
    libqt5gui5 \
    libqt5network5 \
    libqt5opengl5 \
    libqt5widgets5 \
    libqt5xml5 \
    qtbase5-dev \
    qtbase5-dev-tools \
    qtchooser \
    qt5-qmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/robodk-installer
RUN mkdir -p /root/Desktop \
    && apt-get update \
    && curl -fsSL https://cdn.robodk.com/downloads/Install-RoboDK.tar.gz -o Install-RoboDK.tar.gz \
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
        install

WORKDIR /workspace
COPY crxkinematics.pro ./crxkinematics.pro
COPY include ./include
COPY src ./src
COPY third_party/eigen ./third_party/eigen

RUN mkdir -p build \
    && cd build \
    && qmake ../crxkinematics.pro "CONFIG+=release" "ROBODK_DEPLOY=1" "ROBODK_ROOT=/opt/robodk/bin" \
    && make -j"$(nproc)" \
    && test -f /opt/robodk/bin/robotextensions/crx_kinematics.so
    
FROM base AS final

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-keysyms1 \
    libxcb-xinerama0 \
    libxrender1 \
    libqt5core5a \
    libqt5gui5 \
    libqt5network5 \
    libqt5opengl5 \
    libqt5widgets5 \
    libqt5xml5 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/robodk /opt/robodk
COPY .docker/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

RUN chmod +x /usr/local/bin/docker-entrypoint.sh \
    && useradd -ms /bin/bash robodk \
    && mkdir -p /home/robodk/.config/RoboDK \
    && chown -R robodk:robodk /home/robodk /opt/robodk

ENV RDK_PORT=20501 \
    RDK_LIC="" \
    LD_LIBRARY_PATH=/opt/robodk/bin/lib \
    QT_PLUGIN_PATH=/opt/robodk/bin/plugins \
    QT_QPA_PLATFORM_PLUGIN_PATH=/opt/robodk/bin/plugins \
    QT_QPA_PLATFORM=offscreen

EXPOSE 20501
USER robodk
WORKDIR /home/robodk

ENV LD_LIBRARY_PATH="/opt/robodk/bin/lib"
ENV QT_PLUGIN_PATH="/opt/robodk/bin/plugins/"
ENV QT_QPA_PLATFORM_PLUGIN_PATH="/opt/robodk/bin/plugins"

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
