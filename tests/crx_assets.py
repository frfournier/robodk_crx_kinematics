"""Approved model inputs for #19 A; hashes are Git LFS SHA-256 object IDs.

Inventory verified at 2a2227aecd61a2094804117cac13f8ba8cc9eaab. Nominal joint
frames, command probes and limits are captured in fixtures/crx_asset_frames.json.
The Python oracle derives and checks the canonical mapping; runtime integration
and command-limit semantics remain pending. Hashes alone do not establish support.
"""

APPROVED_ROBOT_ASSETS = {
    "Fanuc-CRX-5iA-Custom.robot": "e7c5029cd9a8cc4d8eb8d4acf5860f4ab67e1b1d34ea4af345ad5e14a7f40a2f",
    "Fanuc-CRX-10iA-Custom.robot": "d0eead452ee5d299893dba03bdfbc3d1f751d0b703a32ec5e510e3198242c471",
    "Fanuc-CRX-10iA-L-Custom.robot": "8fd577fded999190e3203f256bd88b3b9b6b5738fc7e4e050cf5d58bdea33557",
    "Fanuc-CRX-20iA-L.robot": "6b27566edae427c6e9c2aac673fbaed5ae6dc4a3fa39aa806e6625b795c2b57c",
    "Fanuc-CRX-25iA.robot": "a4191e63837091a57f0a336cc51a78f19ab36fa08a8c412fc6e65c5416635969",
    "Fanuc-CRX-30iA-Custom.robot": "d72cbd33ae616db178e91c8dcc298cbc9ec66cd388232a808a35efeb8bea715c",
}
