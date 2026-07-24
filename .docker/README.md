# RoboDK CRX kinematics container

This container builds the CRX custom-kinematics library, installs it with the
latest RoboDK Linux release, and exposes the RoboDK API on TCP port `20501`.
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
docker compose -f compose.yaml -f compose.build.yaml build --pull --no-cache
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
docker buildx bake --pull --no-cache --push
```

The Bake target publishes `linux/amd64` and attaches an SBOM and provenance.
RoboDK's unversioned installer URL is fetched without cache so each build uses
the latest available Linux release.

## Refresh RoboDK

Rebuild with `--no-cache` to fetch and install the latest RoboDK release, then
run the smoke tests before publishing.
