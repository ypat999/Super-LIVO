#!/usr/bin/env bash
echo "=== repo info ==="
curl -s https://api.github.com/repos/Haz25/POINT-LIVO | grep -E '"description"|"default_branch"|"language"|"fork"|"stargazers_count"|"pushed_at"'
echo "=== branches ==="
curl -s 'https://api.github.com/repos/Haz25/POINT-LIVO/branches?per_page=30' | grep '"name"'
echo "=== root tree ==="
curl -s "https://api.github.com/repos/Haz25/POINT-LIVO/git/trees/main?recursive=0" | grep -E '"path"' | head -30
