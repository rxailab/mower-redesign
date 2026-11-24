#!/bin/bash
# [fix1] all checks for NAME.npz -> chk/NAME.*   usage: ./checks.sh NAME
cd "$(dirname "$0")"; N=$1; PY=../venv/bin/python; mkdir -p chk
$PY topo.py $N.npz > chk/$N.topo.json 2>&1 &
$PY check_wall.py $N.npz --json chk/$N.wall.json > chk/$N.wall.txt 2>&1 &
$PY wall_clear.py $N.npz dev/base_parts.npz --json chk/$N.clear.json > chk/$N.clear.txt 2>&1 &
$PY check_cover2.py $N.npz > chk/$N.cover2.txt 2>&1 &
$PY wallprobe.py $N.npz > chk/$N.probe.txt 2>&1 &
$PY kitcheck.py $N.npz chk/$N.kit.json > chk/$N.kit.txt 2>&1 &
$PY kitclear.py $N.npz $N.json --json chk/$N.kitclear.json > chk/$N.kitclear.txt 2>&1 &      # [fix4-cover2] G1: cover kit vs the real cut chassis over the lift (>= 0.5)
$PY labthermal.py $N.npz $N.check.json chk/$N.thermal.json > chk/$N.thermal.txt 2>&1 &      # [fix4-cover2] M5: labyrinth / deck clearances at -0.56e-3 .. +1.47e-3 relative strain
wait
$PY lidsim.py $N.npz chk/$N.sim.json 12 > chk/$N.sim.txt 2>&1
$PY lidsim_ctl.py $N.npz > chk/$N.ctl.txt 2>&1
$PY interfere.py $N.npz --quiet --json chk/$N.interf.json > chk/$N.interf.txt 2>&1      # [fix4-qa] pairwise interference matrix (Blender)
$PY solidaudit.py $N.npz --json chk/$N.solid.json > chk/$N.solid.txt 2>&1      # [fix4-qa] closed / oriented / self-intersection per component
echo CHECKS DONE $N
