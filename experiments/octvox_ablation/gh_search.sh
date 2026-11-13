#!/usr/bin/env bash
curl -s 'https://api.github.com/search/repositories?q=point_lio&sort=stars&order=desc&per_page=20' \
  | grep -E '"full_name"|"stargazers_count"|"description"' | head -70
