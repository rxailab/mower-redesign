#!/bin/bash
# [hz] full check suite on h.npz (one heavy job at a time)
cd "$(dirname "$0")"; PY=../venv/bin/python; L=../../heavy.lock; mkdir -p chk
flock $L ./checks.sh h; echo "checks.sh done"
flock $L $PY lifttest.py h.npz > chk/h.lift.txt 2>&1; echo "lifttest $?"
flock $L $PY deckcheck.py h.npz dev/base_parts.npz > chk/h.deck.txt 2>&1; echo "deckcheck $?"
flock $L $PY stopcheck.py h.npz h.json chk/h.stop.json > chk/h.stop.txt 2>&1; echo "stopcheck $?"
flock $L $PY strutcheck.py h.npz dev/base_parts.npz chk/h.struts.json > chk/h.struts.txt 2>&1; echo "strutcheck $?"
flock $L $PY portcheck.py h.npz h.json --json chk/h.ports.json > chk/h.ports.txt 2>&1; echo "portcheck $?"
flock $L $PY labcheck.py h.npz chk/h.lab.json --step 0.25 > chk/h.lab.txt 2>&1; echo "labcheck $?"
echo ALL DONE
