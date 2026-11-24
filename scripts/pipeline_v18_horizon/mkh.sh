#!/bin/bash
# [hz] build the Horizon parameter files: h.json (full) and h_web.json (web res). usage: ../mkh.sh   (inside design/)
cd "$(dirname "$0")/design"; F="fix3_labyrinth.json fix3_struts.json fix3_ports.json fix3_lights.json fix3_stop.json fix3_hinge.json fix4_stop2.json fix4_hinge2.json fix4_front2.json fix4_cover2.json fix4_int.json horizon.json $(ls hz_*.json 2>/dev/null | tr '\n' ' ')"
../venv/bin/python mk_f3.py h.json $F > /dev/null && ../venv/bin/python mk_f3.py h_web.json $F web.json > /dev/null && echo "h.json h_web.json <- $F"
