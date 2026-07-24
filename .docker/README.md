# RoboDK CRX kinematics container

This container builds the CRX custom-kinematics library, installs it in a
headless RoboDK 6.0.6 image, and exposes the RoboDK API on TCP port `20501`.
It runs as an unprivileged user with a read-only root filesystem.

RoboDK is proprietary software. Review the [RoboDK EULA](https://robodk.com/eula)
and confirm that your license permits building and distributing this image
before publishing it.

Run all commands in this directory. Copy `.env.example` to `.env` first if you
want to customize the defaults.

## Build and run locally

Setting `ROBODK_ACCEPT_EULA=yes` confirms that you reviewed and accept RoboDK's
license terms:

```powershell
$env:ROBODK_ACCEPT_EULA = "yes"
docker compose -f compose.yaml -f compose.build.yaml build --pull
docker compose -f compose.yaml -f compose.build.yaml up --detach
docker compose ps
```

The API is available at `127.0.0.1:20501` by default. Set
`ROBODK_BIND_ADDRESS=0.0.0.0` only when remote access is intended, and protect
the port with an appropriate firewall.

RoboDK stations and generated files are stored in `./data`. User configuration
is stored in the `robodk-config` named volume.

## Add a license securely

Place only the license value in `secrets/robodk-license.txt`, then include the
license overlay:

```powershell
docker compose `
  -f compose.yaml `
  -f compose.build.yaml `
  -f compose.license.yaml `
  up --detach
```

The ignored `secrets/` directory prevents accidental inclusion in the build
context or source control. The secret is mounted read-only at runtime and is
never stored in the image or Compose environment.

## Publish with Buildx Bake

Authenticate, choose the registry image name, and publish versioned and
`latest` tags:

```powershell
docker login
$env:IMAGE_NAME = "your-dockerhub-user/robodk-crx-kinematics"
$env:CRX_KINEMATICS_VERSION = "0.3.1"
docker buildx bake --pull --push
```

The Bake target publishes `linux/amd64`, attaches an SBOM and provenance, and
pins the RoboDK installer by SHA-256. If RoboDK replaces the download, the
build fails instead of silently producing a different image.

## Refresh RoboDK

When RoboDK publishes a new installer:

1. Download `https://cdn.robodk.com/downloads/Install-RoboDK.tar.gz`.
2. Calculate its SHA-256 (`Get-FileHash` on PowerShell or `sha256sum` on Linux).
3. Update `ROBODK_VERSION` and `ROBODK_DOWNLOAD_SHA256` in `docker-bake.hcl`.
4. Update the matching defaults in `Dockerfile`, `compose.build.yaml`, and
   `.env.example`.
5. Build without cache and smoke-test before publishing.

The current pinned installer was downloaded on 2026-07-23 and has SHA-256
`0d965236db076d35f9337a93b3ee79dc75a361383f63ca230f33bd2658b55701`.
