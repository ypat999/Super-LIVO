#!/usr/bin/env bash
for R in Haz25/POINT-LIVO S-Torin/Point-line-LIVO; do
  echo "########## $R ##########"
  curl -s "https://api.github.com/repos/$R" | grep -E '"description"|"default_branch"|"fork"|"stargazers_count"|"pushed_at"|"message"'
  echo "--- branches ---"
  curl -s "https://api.github.com/repos/$R/branches?per_page=30" | grep '"name"'
done
