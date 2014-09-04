# -*- coding: utf-8 -*-
"""
Created on Tue Aug 26 14:22:05 2014

@author: user
"""

import socket
import struct

MCAST_GRP = '224.0.1.133'
MCAST_PORT = 2986
IF_ADDR = "10.0.0.2"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((IF_ADDR, MCAST_PORT))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(IF_ADDR))

mreq = socket.inet_aton(MCAST_GRP) + socket.inet_aton("0.0.0.0") #IF_ADDR)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
  print sock.recv(10240)
