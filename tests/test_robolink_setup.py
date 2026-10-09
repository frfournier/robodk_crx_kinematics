"""Check live-test prerequisites without launching or modifying RoboDK."""

import sys

import pytest

import test_crx_robolink_kinematics as integration


@pytest.mark.skipif(sys.platform != "win32", reason="Windows DLL deployment guard")
@pytest.mark.parametrize("installed", [None, b"stale", b"current"])
def test_live_test_setup_never_deploys(tmp_path, monkeypatch, installed):
    repo = tmp_path / "repo"
    build_dll = repo / "build" / "Release" / "crx_kinematics.dll"
    build_dll.parent.mkdir(parents=True)
    build_dll.write_bytes(b"current")
    robodk_root = tmp_path / "RoboDK"
    deployed_dll = robodk_root / "bin" / "robotextensions" / build_dll.name
    if installed is not None:
        deployed_dll.parent.mkdir(parents=True)
        deployed_dll.write_bytes(installed)

    monkeypatch.setattr(integration, "__file__", str(repo / "tests" / "live.py"))
    monkeypatch.setattr(integration, "_resolve_robodk_root", lambda: robodk_root)

    if installed == b"current":
        integration._require_robolink_extension_or_skip()
    else:
        with pytest.raises(pytest.skip.Exception, match="Deploy the matching DLL"):
            integration._require_robolink_extension_or_skip()

    if installed is None:
        assert not robodk_root.exists()
    else:
        assert deployed_dll.read_bytes() == installed
