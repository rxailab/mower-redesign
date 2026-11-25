#!/bin/bash
cd "$(dirname "$0")/.."; PY=../venv/bin/python; L=../../heavy.lock
flock $L $PY design_gen.py f9.json f9.npz > f9.check.json 2> f9.err; echo "gen $?"
$PY design_gen.py f9_web.json f9_web.npz > f9_web.check.json 2> f9_web.err; echo "gen web $?"
flock $L ./checks.sh f9; echo "checks.sh done"
flock $L $PY lifttest.py f9.npz > chk/f9.lift.txt 2>&1; echo "lift $?"
flock $L $PY deckcheck.py f9.npz dev/base_parts.npz > chk/f9.deck.txt 2>&1; echo "deck $?"
flock $L $PY stopcheck.py f9.npz f9.json chk/f9.stop.json > chk/f9.stop.txt 2>&1; echo "stop $?"
flock $L $PY strutcheck.py f9.npz dev/base_parts.npz chk/f9.struts.json > chk/f9.struts.txt 2>&1; echo "struts $?"
flock $L $PY portcheck.py f9.npz f9.json --json chk/f9.ports.json > chk/f9.ports.txt 2>&1; echo "ports $?"
flock $L $PY lightcheck.py f9.npz dev/base_parts.npz > chk/f9.lights.txt 2>&1; echo "lights $?"
flock $L $PY labcheck.py f9.npz chk/f9.lab.json --step 0.25 > chk/f9.lab.txt 2>&1; echo "lab $?"
flock $L ~/blender-4.2.3-linux-x64/blender -b --factory-startup -P rimfold.py -- f9.npz chk/f9.rimfold.json > chk/f9.rimfold.txt 2>&1; echo "rimfold $?"
echo ALL DONE
