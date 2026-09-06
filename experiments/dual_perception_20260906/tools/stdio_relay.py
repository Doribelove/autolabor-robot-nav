#!/usr/bin/env python3
"""SSH transports RGB tensors; loopback service owns both resident models."""
import os
import select
import socket
import sys

with socket.create_connection(('127.0.0.1',11681),timeout=4) as connection:
    connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    connection.settimeout(None)
    while True:
        ready,_,_=select.select([sys.stdin.buffer,connection],[],[],15)
        if not ready:continue
        if sys.stdin.buffer in ready:
            data=os.read(sys.stdin.fileno(),65536)
            if not data:break
            connection.sendall(data)
        if connection in ready:
            data=connection.recv(65536)
            if not data:break
            view=memoryview(data)
            while view:view=view[os.write(sys.stdout.fileno(),view):]
