#!/usr/bin/env bash
# 下载 RTK-SLAM Dataset construction_seq2 (ROS2 .db3, 期望 10,656,112,640 B)。
# 依次尝试 hf-mirror 与 huggingface.co（mirror 的 resolve 端点会返回 CF 521 HTML，
# 故必须校验内容而非只看 curl 退出码），支持断点续传。
D=/home/ywj/slam_data/rtk_slam/construction_seq2
P=datasets/Willyzw/rtk-slam-dataset/resolve/main/ros2/construction_seq2
EXPECT=10656112640
cd "$D" || exit 1

# 清掉之前误存的 HTML 错误页
for f in metadata.yaml construction_seq2.db3; do
  if [ -f "$f" ] && head -c 200 "$f" | grep -qi '<!DOCTYPE\|cloudflare'; then
    echo "删除 HTML 垃圾: $f"; rm -f "$f"
  fi
done

fetch() {  # $1=文件名 $2=期望大小(0=不校验)
  for m in https://hf-mirror.com/$P https://huggingface.co/$P; do
    echo ">>> $1 via $m"
    curl -L -C - --retry 10 --retry-delay 15 --retry-all-errors \
         --connect-timeout 20 --speed-limit 200000 --speed-time 60 \
         -o "$1" "$m/$1"
    rc=$?
    if head -c 300 "$1" | grep -qi '<!DOCTYPE\|cloudflare'; then
      echo "!!! $1 内容是 HTML 错误页 (rc=$rc)，删除换源"; rm -f "$1"; continue
    fi
    if [ "$2" != 0 ]; then
      sz=$(stat -c %s "$1" 2>/dev/null || echo 0)
      if [ "$sz" != "$2" ]; then echo "!!! $1 大小 $sz != $2 (rc=$rc)，续传重试"; continue; fi
    fi
    echo "OK $1"; return 0
  done
  return 1
}

fetch metadata.yaml 0 || { echo "metadata 下载失败"; exit 2; }
echo "--- metadata.yaml ---"; cat metadata.yaml

fetch construction_seq2.db3 "$EXPECT" || { echo "db3 下载失败"; exit 3; }
ls -la "$D"
sha256sum construction_seq2.db3 | tee construction_seq2.db3.sha256
echo "=== DONE ==="
