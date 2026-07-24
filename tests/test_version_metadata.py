import re
import tomllib
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_version_metadata_is_consistent():
    with (REPO_ROOT / "pyproject.toml").open("rb") as stream:
        version = tomllib.load(stream)["project"]["version"]
    version_major, version_minor, version_patch = version.split(".")

    cmake = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    qmake = (REPO_ROOT / "crxkinematics.pro").read_text(encoding="utf-8")
    public_header = (REPO_ROOT / "include" / "crx_kinematics.h").read_text(
        encoding="utf-8"
    )
    readme = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
    changelog = (REPO_ROOT / "CHANGELOG.md").read_text(encoding="utf-8")

    assert re.search(
        rf"project\(crx_kinematics VERSION {re.escape(version)} LANGUAGES CXX\)",
        cmake,
    )
    assert re.search(rf"^VERSION = {re.escape(version)}$", qmake, re.MULTILINE)
    assert (
        f'#define CRX_KINEMATICS_VERSION_STRING "{version}"' in public_header
    )
    assert (
        f"#define CRX_KINEMATICS_VERSION_MAJOR {version_major}" in public_header
    )
    assert (
        f"#define CRX_KINEMATICS_VERSION_MINOR {version_minor}" in public_header
    )
    assert (
        f"#define CRX_KINEMATICS_VERSION_PATCH {version_patch}" in public_header
    )
    assert f"**Current release:** `{version}`" in readme
    assert f"## {version}" in changelog
