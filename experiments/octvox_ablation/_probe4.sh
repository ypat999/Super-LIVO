#!/usr/bin/env bash
echo "=== lo link ==="
ip link show lo
echo "=== addr ==="
ip -4 addr show lo
echo "=== rmw impls ==="
ls /opt/ros/humble/lib | grep -i rmw | head
echo "=== cyclonedds? ==="
ls /opt/ros/humble/lib/librmw_cyclonedds_cpp.so 2>&1
echo "=== sudo nopasswd? ==="
sudo -n true 2>&1 && echo YES || echo NO
echo "=== uptime ==="
uptime
