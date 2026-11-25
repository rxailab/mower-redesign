#!/bin/bash
cd "$(dirname "$0")/.."; PY=../venv/bin/python; L=../../heavy.lock
flock $L $PY design_gen.py f6.json f6.npz > f6.check.json 2> f6.err; echo "gen $?"
$PY design_gen.py f6_web.json f6_web.npz > f6_web.check.json 2> f6_web.err; echo "gen web $?"
flock $L ./checks.sh f6; echo "checks.sh done"
flock $L $PY lifttest.py f6.npz > chk/f6.lift.txt 2>&1; echo "lift $?"
flock $L $PY deckcheck.py f6.npz dev/base_parts.npz > chk/f6.deck.txt 2>&1; echo "deck $?"
flock $L $PY stopcheck.py f6.npz f6.json chk/f6.stop.json > chk/f6.stop.txt 2>&1; echo "stop $?"
flock $L $PY strutcheck.py f6.npz dev/base_parts.npz chk/f6.struts.json > chk/f6.struts.txt 2>&1; echo "struts $?"
flock $L $PY portcheck.py f6.npz f6.json --json chk/f6.ports.json > chk/f6.ports.txt 2>&1; echo "ports $?"
flock $L $PY lightcheck.py f6.npz dev/base_parts.npz > chk/f6.lights.txt 2>&1; echo "lights $?"
flock $L $PY labcheck.py f6.npz chk/f6.lab.json --step 0.25 > chk/f6.lab.txt 2>&1; echo "lab $?"
flock $L ~/blender-4.2.3-linux-x64/blender -b --factory-startup -P rimfold.py -- f6.npz chk/f6.rimfold.json > chk/f6.rimfold.txt 2>&1; echo "rimfold $?"
echo ALL DONE
