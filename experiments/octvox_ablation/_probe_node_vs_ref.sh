#!/usr/bin/env bash
set -e
cd /home/ywj/git/Super-LIVO
/usr/bin/time -v python3 experiments/octvox_ablation/_probe_node_vs_ref.py 2>&1 | grep -v "^\s*\(Page\|File\|Socket\|Signal\|Swaps\|Average\|Voluntary\|Involuntary\|Exit\|Minor\|Major\|Percent\|System time\|User time\)" || true
