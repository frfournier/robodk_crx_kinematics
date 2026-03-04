FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git qt5-qmake qtbase5-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /src