# Reconstructed nominal models

The recovery re-fetched and pinned reviewed public sources from:

- UniversalRobots/Universal_Robots_ROS2_Description, UR5e nominal configuration.
- ros-industrial/abb, ABB IRB120 3/0.58 macro.
- ros-industrial/fanuc, FANUC LR Mate 200iD macro.

Exact commits, URLs and hashes are in upstream/manifest.json; upstream authors,
licenses and package metadata remain beside their files. These are newly pinned
recovery inputs; equality to the inaccessible former manifest is not asserted.

`tools/models/generate_reference_models.py` performs only reviewed scalar expansion and
whitelisted AST arithmetic; it never runs Xacro/ROS, eval, external expressions
or network requests. It creates expanded geometry-only URDFs, SI YAML profiles,
independent homogeneous-matrix FK fixtures and the three example programs.
Regeneration requires optional PyYAML. Normal C++ building does not.

The zero TCP positions match the values recorded in the earlier conversation:
UR5e approximately (0.8172,0.2329,0.0628) m; ABB (0.374,0,0.63) m;
FANUC (0.465,0,0.695) m. This is nominal-model agreement, not physical calibration.
No inertias, certified dynamics, tool/payload or drive scaling are inferred.
