# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2026 Centre Technologique en Aérospatiale
"""Approved model inputs for #19 A; hashes are Git LFS SHA-256 object IDs.

Inventory verified at 0df62df (updated custom-kinematics assets). Nominal joint
frames, command probes and limits are captured in fixtures/crx_asset_frames.json.
The Python oracle derives and checks the canonical mapping. Live RoboDK 6.1.1
tracing verifies all six assets invoke this DLL. Hashes alone do not establish support.
"""

APPROVED_ROBOT_ASSETS = {
    "Fanuc-CRX-5iA-Custom.robot": "a7ad092a220ab60b3d1a2da58a4bf741d30a455187f104ba5a356e3d56cf0e7b",
    "Fanuc-CRX-10iA-Custom.robot": "42f085ae5eeecb614d8e2b2184c29d3e3f5969c5da685c8c84b03381cb9f229d",
    "Fanuc-CRX-10iA-L-Custom.robot": "cc004327ff060369b985877c5a1022534577f6a5c5eb9d147da59af4f06aa030",
    "Fanuc-CRX-20iA-L-Custom.robot": "c58e44e8ad8f1fc21af345368b2b70c79375064aa5e58075dbce44dcd7650fbc",
    "Fanuc-CRX-25iA-Custom.robot": "e1a2747a40cf51fbf9f29d81c3e29a87a0fd65a06088004d1ec7c4624d6ef7dd",
    "Fanuc-CRX-30iA-Custom.robot": "9365d088d39795a76efad7a096efffae3749989a2e05d9f007887caab72892f4",
}
