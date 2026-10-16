"""Approved model inputs for #19 A; hashes are Git LFS SHA-256 object IDs.

Inventory verified at 0df62df (updated custom-kinematics assets). Nominal joint
frames, command probes and limits are captured in fixtures/crx_asset_frames.json.
The Python oracle derives and checks the canonical mapping. Live RoboDK 6.1.1
tracing verifies all six assets invoke this DLL. Hashes alone do not establish support.
"""

APPROVED_ROBOT_ASSETS = {
    "Fanuc-CRX-5iA-Custom.robot": "f08d026058b13455ebbf7b1534a2f6d03cb09b8dd80dd4ea0d69d069d6ee256a",
    "Fanuc-CRX-10iA-Custom.robot": "fb3e324252c9be3da14104593f773d7e54bd057ad7f933bf5686d8a7bef41f4c",
    "Fanuc-CRX-10iA-L-Custom.robot": "479984b2db5f0872bd766bb1af61849278237a245b0e367742e7ad5ef7e1d1d6",
    "Fanuc-CRX-20iA-L-Custom.robot": "caba1d97c6eaa31f776b864ea0e7dc63d1362225192e728c5d6f2cf8a297e6c3",
    "Fanuc-CRX-25iA-Custom.robot": "ecf9eba17e1520543a134fec9096ff105480dec9a3e2de0b3e0727b8470cc758",
    "Fanuc-CRX-30iA-Custom.robot": "2d629cc514b0a2d23884b9299683a9f52590780efd421d94cabca8061ef7b6db",
}
