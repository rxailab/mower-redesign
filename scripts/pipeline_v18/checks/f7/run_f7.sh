#!/bin/bash
cd "$(dirname "$0")/.."; PY=../venv/bin/python; L=../../heavy.lock
flock $L $PY design_gen.py f7.json f7.npz > f7.check.json 2> f7.err; echo "gen $?"
$PY design_gen.py f7_web.json f7_web.npz > f7_web.check.json 2> f7_web.err; echo "gen web $?"
flock $L ./checks.sh f7; echo "checks.sh done"
flock $L $PY lifttest.py f7.npz > chk/f7.lift.txt 2>&1; echo "lift $?"
flock $L $PY deckcheck.py f7.npz dev/base_parts.npz > chk/f7.deck.txt 2>&1; echo "deck $?"
flock $L $PY stopcheck.py f7.npz f7.json chk/f7.stop.json > chk/f7.stop.txt 2>&1; echo "stop $?"
flock $L $PY strutcheck.py f7.npz dev/base_parts.npz chk/f7.struts.json > chk/f7.struts.txt 2>&1; echo "struts $?"
flock $L $PY portcheck.py f7.npz f7.json --json chk/f7.ports.json > chk/f7.ports.txt 2>&1; echo "ports $?"
flock $L $PY lightcheck.py f7.npz dev/base_parts.npz > chk/f7.lights.txt 2>&1; echo "lights $?"
flock $L $PY labcheck.py f7.npz chk/f7.lab.json --step 0.25 > chk/f7.lab.txt 2>&1; echo "lab $?"
flock $L ~/blender-4.2.3-linux-x64/blender -b --factory-startup -P rimfold.py -- f7.npz chk/f7.rimfold.json > chk/f7.rimfold.txt 2>&1; echo "rimfold $?"
echo ALL DONE
