#!/usr/bin/env bash
echo "=== search: POINT-LIVO / FAST-LIVO2 ros2 forks ==="
curl -s 'https://api.github.com/search/repositories?q=LIVO+ros2&sort=stars&order=desc&per_page=15' | grep -E '"full_name"|"stargazers_count"|"description"' | head -40
echo "=== search: fast_livo humble ==="
curl -s 'https://api.github.com/search/repositories?q=FAST-LIVO2&sort=stars&order=desc&per_page=10' | grep -E '"full_name"|"stargazers_count"|"description"' | head -30
echo "=== forks of S-Torin/Point-line-LIVO ==="
curl -s 'https://api.github.com/repos/S-Torin/Point-line-LIVO/forks?per_page=20' | grep '"full_name"' | head -20
echo "=== forks of Haz25/POINT-LIVO ==="
curl -s 'https://api.github.com/repos/Haz25/POINT-LIVO/forks?per_page=20' | grep '"full_name"' | head -20
