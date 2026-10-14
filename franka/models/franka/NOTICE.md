# Panda model attribution and provenance

The 67 unmodified collision/visual mesh files in `assets/` are byte-for-byte copies of
Google DeepMind MuJoCo Menagerie, `franka_emika_panda/assets`, at revision
`0059d4335f8156206f63a35662313385f7ad6d74` (2026-10-07).

Source: https://github.com/google-deepmind/mujoco_menagerie/tree/0059d4335f8156206f63a35662313385f7ad6d74/franka_emika_panda

The upstream model is explicitly released under Apache-2.0; its exact `LICENSE`
is retained here. Menagerie attribution: Copyright 2022 DeepMind Technologies
Limited, with contributions by MuJoCo Menagerie contributors. The model derives
from Franka Emika's publicly available `franka_ros/franka_description` URDF and
meshes, as described in the preserved `UPSTREAM_README.md`.

`ASSET_PROVENANCE.json` records SHA-256 for every local mesh and the license.
Verification compared all 67 files against a sparse checkout at the exact revision
above: 67 matched, zero mismatched or missing. Recheck local integrity with
`./.venv/bin/python franka/scripts/verify_assets.py` from the repository root.

Local XML adaptations include the task-world Panda base translation, the fingertip
`ee_site`, scene/table/cube placement, and compiler/asset paths for local includes.
The optional cube+plate example's machine-specific include has been changed to a
relative external dependency path. These XML modifications are local Lab 3 work;
the mesh bytes have not been altered. The optional plate dependency is absent and
is outside the validated scene and dashboard workflow.

Keep this attribution, upstream model README/changelog, and the Apache license
when redistributing the bundled model. No upstream NOTICE file was present in the
Panda model directory at the verified revision.
