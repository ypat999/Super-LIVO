#!/bin/bash
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO || exit 1
echo "=== local branches ==="
git branch -vv | cat
echo "=== stash ==="
git stash list | cat
echo "=== 哪些分支/提交含 g_visual_enable ==="
git log --all --oneline -S 'g_visual_enable' -- src/super_lio/include/lio/params.h | cat
echo "=== 工作区 visual 相关文件（新增未跟踪/已跟踪）==="
git status --short | grep -i visual | head -20
echo "=== 未跟踪文件数 ==="
git ls-files --others --exclude-standard | wc -l
