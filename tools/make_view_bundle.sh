#!/usr/bin/env bash
# make_view_bundle.sh - one folder for viewing the arm on another computer: the scene model (the URDF
# converted with the visual meshes), the meshes, and tools/sim_view.py.
#
#     bash tools/make_view_bundle.sh OUT_DIR MESH_DIR
#
# MESH_DIR: the visual meshes of the Indy7 (Indy7_0.stl ... Indy7_6.stl), from the ROS 2 package
# indy_description of neuromeka-robotics/indy-ros2 (meshes/indy7/visual). Copy OUT_DIR to the other
# computer and run there:
#   pip install mujoco numpy pillow          (Python >= 3.10)
#   python sim_view.py live indy7_scene.xml --host <address of the controller>
#   python sim_view.py replay indy7_scene.xml <jt_*.csv or hl_*.csv>
#   python sim_view.py render indy7_scene.xml --q "0 0 -1.5708 0 -1.5708 0" --out fig.png
#   python sim_view.py record indy7_scene.xml <hl_jump_*.csv> --out jump.mp4
#        (pip install imageio imageio-ffmpeg; on a Linux without a display: MUJOCO_GL=egl)
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
if [ $# -ne 2 ]; then
    sed -n '2,15p' "$0"
    exit 2
fi
OUT=$1
MESH=$2
PY=${PYTHON:-python3}
[ -d "$MESH" ] || { echo "no mesh directory $MESH" >&2; exit 1; }

mkdir -p "$OUT/meshes"
"$PY" "$REPO/tools/urdf2mjcf.py" "$REPO/indy7/models/indy7.urdf" "$OUT/indy7_scene.xml" --scene --meshdir meshes
# copy exactly the mesh files that the scene uses
for f in $(grep -o 'file="[^"]*"' "$OUT/indy7_scene.xml" | sed 's/file="\(.*\)"/\1/'); do
    cp "$MESH/$f" "$OUT/meshes/"
done
cp "$REPO/tools/sim_view.py" "$OUT/"
echo "bundle: $OUT ($(ls "$OUT/meshes" | wc -l) meshes, $(du -sh "$OUT" | cut -f1))"
