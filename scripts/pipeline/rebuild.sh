set -e
cd "$(dirname "$0")"
PY=../venv/bin/python
OMP_NUM_THREADS=8 $PY cleanmid4.py
OMP_NUM_THREADS=8 $PY label_mid.py
$PY make_decals.py
ROOT=$(cd ../.. && pwd)
~/blender-4.2.3-linux-x64/blender -b --factory-startup -P $ROOT/scripts/export_glb.py -- $PWD $ROOT/viewer/models 2>&1 | grep -E "faces|exported|Error|Traceback|line [0-9]+"
$PY make_hotspots.py $ROOT/viewer/models
echo REBUILT
