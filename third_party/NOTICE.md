# Third-party dependencies and model sources

The reconstruction bundles unmodified source archives for offline builds:
Eigen 3.4.0, yaml-cpp 0.8.0, TinyXML2 11.0.0 and optional Ruckig Community v0.15.3.
Each archive's SHA256 is pinned in CMakeLists.txt. Licenses are retained inside
those archives and copied into licenses/. Ruckig provenance includes the exact
upstream commit in ruckig-provenance.json; its archive is a normalized tar of that
unmodified tagged checkout, excluding .git.

The independent FK/J tests additionally build a subset of **Orocos KDL 1.5.1**
(LGPL 2.1 or later). Its complete, unmodified upstream source archive is bundled;
the exact tag commit and SHA256 are in kdl-provenance.json and
test_support/kinematics/kdl/CMakeLists.txt. COPYING is retained in the archive and copied
into licenses/orocos-kdl-1.5.1-COPYING. KDL links only into the reference test,
not the motion libraries or simulator. MOTION_WITH_KDL_REFERENCE=OFF omits it.

The nominal UR5e / ABB IRB120 / FANUC LR Mate 200iD descriptions are modified
kinematics-only derivations of the pinned upstream ROS descriptions in
../models/upstream. Original copyright/license notices and package metadata are
preserved. ../models/upstream/manifest.json records each source URL, commit and
file checksum. Generated fixtures omit visual meshes, inertias and drive setup;
example acceleration/jerk caps are prototype choices, not manufacturer ratings.

Do not infer calibrated robot accuracy, supported drive units or physical safe
limits from the nominal references. Python dependencies are optional tooling only.
