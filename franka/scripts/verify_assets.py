"""Verify the bundled Panda mesh and license hashes against the pinned manifest."""
import hashlib
import json
from pathlib import Path

model = Path(__file__).resolve().parents[1] / "models" / "franka"
manifest = json.loads((model / "ASSET_PROVENANCE.json").read_text())
failures = []
for asset in manifest["assets"]:
    path = model / "assets" / asset["file"]
    if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != asset["sha256"]:
        failures.append(asset["file"])
license_path = model / "LICENSE"
if hashlib.sha256(license_path.read_bytes()).hexdigest() != manifest["license_sha256"]:
    failures.append("LICENSE")
if failures:
    raise SystemExit("Asset verification failed: " + ", ".join(failures))
print(json.dumps({"status": "passed", "meshes_verified": len(manifest["assets"]),
                  "upstream_revision": manifest["revision"], "license": "Apache-2.0"}, indent=2))
