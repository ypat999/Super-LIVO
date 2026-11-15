#!/usr/bin/env python3
"""跨进程 UDP 单播 / 多播连通性测试。"""
import socket
import subprocess
import sys
import time

PORT = 47566
MODE = sys.argv[1] if len(sys.argv) > 1 else "server"

if MODE == "server":
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("192.168.1.5", PORT))
    s.settimeout(8)
    try:
        d, a = s.recvfrom(128)
        print("UNICAST RECV OK:", d, a)
    except socket.timeout:
        print("UNICAST RECV TIMEOUT")
else:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    time.sleep(1.0)
    s.sendto(b"ping", ("192.168.1.5", PORT))
    print("sent ping")
