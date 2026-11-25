#!/bin/bash
# usage: rv.sh PARAMS NPZ OUTPREFIX VIEWS RES SPP   (env CUSTOM_VIEWS, WEBBG, LAYER, LIDMAP, MATVIEWS, HEADLIGHT ... pass through to design_render.py)
# Runs scripts/pipeline_v18/design_render.py (or $P); waits while 2 or more Blender processes run.
HERE=$(cd "$(dirname "$0")" && pwd); P=${P:-$HERE/../pipeline_v18}; BLENDER=${BLENDER:-~/blender-4.2.3-linux-x64/blender}
O=$3; case "$O" in /*) ;; *) O=$PWD/$O;; esac
cd "$P" || exit 1
while [ "$(pgrep -xc blender)" -ge 2 ]; do sleep 3; done
"$BLENDER" -b --factory-startup -P design_render.py -- "$1" "$2" "$O" "${@:4}" 2>&1 | grep -E "Error|Traceback|rror:|Saved" | tail -40
