#!/usr/bin/env python3
"""检查 WSL 下 loopback 多播是否可用（FastDDS 发现依赖它）。"""
import socket
import struct
import threading
import time

GROUP = "239.255.0.1"
PORT = 47555

def recv():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", PORT))
    mreq = socket.inet_aton(GROUP) + socket.inet_aton("127.0.0.1")
    try:
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    except OSError as e:
        print("JOIN FAIL:", e)
        return
    s.settimeout(3.0)
    try:
        data, addr = s.recvfrom(64)
        print("RECV OK:", data, addr)
    except socket.timeout:
        print("RECV TIMEOUT (多播不可达)")

t = threading.Thread(target=recv, daemon=True)
t.start()
time.sleep(0.5)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))
s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
s.sendto(b"hello", (GROUP, PORT))
t.join(5)

print("--- lo flags ---", open("/sys/class/net/lo/flags").read().strip())
