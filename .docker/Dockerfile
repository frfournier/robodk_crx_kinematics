# syntax=docker/dockerfile:1
FROM ubuntu:24.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

FROM base AS robodk-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
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

WORKDIR /tmp/robodk-installer
RUN mkdir -p /root/Desktop \
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
        install \
    && cd / \
    && rm -rf /tmp/robodk-installer

FROM robodk-base AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    qtbase5-dev \
    qtbase5-dev-tools \
    qtchooser \
    qt5-qmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY crxkinematics.pro ./crxkinematics.pro
COPY include ./include
COPY src ./src
COPY third_party/eigen ./third_party/eigen

RUN mkdir -p build \
    && cd build \
    && qmake ../crxkinematics.pro "CONFIG+=release" \
    && make -j"$(nproc)" \
    && test -f /workspace/build/Release/libcrx_kinematics.so

FROM robodk-base AS runtime

COPY --from=builder /workspace/build/Release/libcrx_kinematics.so /opt/robodk/bin/robotextensions/libcrx_kinematics.so
COPY .docker/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

RUN chmod +x /usr/local/bin/docker-entrypoint.sh \
    && useradd -ms /bin/bash robodk \
    && mkdir -p /home/robodk/.config/RoboDK /opt/robodk/bin/robotextensions \
    && chown -R robodk:robodk /home/robodk /opt/robodk

ENV LD_LIBRARY_PATH=/opt/robodk/bin/lib \
    QT_PLUGIN_PATH=/opt/robodk/bin/plugins \
    QT_QPA_PLATFORM_PLUGIN_PATH=/opt/robodk/bin/plugins \
    QT_QPA_PLATFORM=offscreen

EXPOSE 20501
USER robodk
WORKDIR /home/robodk

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
