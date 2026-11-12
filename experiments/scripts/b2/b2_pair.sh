#!/usr/bin/env bash
bash /tmp/b2_one.sh b2_s010 0.01 2>&1 | grep -vE '^\s+[0-9]+\.[0-9]\s+-' 
bash /tmp/b2_one.sh b2_s005d 0.005 2>&1 | grep -vE '^\s+[0-9]+\.[0-9]\s+-'
