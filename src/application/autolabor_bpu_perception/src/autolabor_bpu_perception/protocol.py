"""Bounded binary protocol for loopback inference and an SSH stdio relay.

Only raw tensors cross this interface. There is no executable path, shell
command, deserialization of Python objects, or robot-control operation.
"""
import json
import math
import os
import select
import socket
import struct
import subprocess
import time

PREFIX = struct.Struct('!4sII')
MAGIC = b'ABP1'
MAX_JSON = 256*1024
MAX_PAYLOAD = 300000*5*4
PORT = 11681


def _read(stream, count, deadline):
    os.set_blocking(stream.fileno(), False)
    chunks = []
    while count:
        remaining = deadline-time.monotonic()
        if remaining <= 0 or not select.select([stream], [], [], max(0,remaining))[0]:
            raise TimeoutError('Inference transport read deadline')
        try:
            chunk = os.read(stream.fileno(), min(count,1024*1024))
        except BlockingIOError:
            continue
        if not chunk:
            raise EOFError('Inference transport closed')
        chunks.append(chunk); count -= len(chunk)
    return b''.join(chunks)


def receive(stream, timeout=2.0):
    deadline = time.monotonic()+timeout
    magic, header_bytes, payload_bytes = PREFIX.unpack(_read(stream,PREFIX.size,deadline))
    if magic != MAGIC or not 2 <= header_bytes <= MAX_JSON or payload_bytes > MAX_PAYLOAD:
        raise ValueError('Invalid inference message size or magic')
    def bad_constant(value):
        raise ValueError('Nonfinite JSON constant: '+value)
    header = json.loads(_read(stream,header_bytes,deadline).decode(),parse_constant=bad_constant)
    if not isinstance(header,dict):
        raise ValueError('Expected JSON object')
    return header, _read(stream,payload_bytes,deadline)


def send(stream, header, payload=b'', timeout=2.0):
    os.set_blocking(stream.fileno(), False)
    raw = json.dumps(header,allow_nan=False,separators=(',',':')).encode()
    if len(raw)>MAX_JSON or len(payload)>MAX_PAYLOAD:
        raise ValueError('Oversized inference message')
    deadline=time.monotonic()+timeout
    for part in (PREFIX.pack(MAGIC,len(raw),len(payload))+raw,payload):
        view=memoryview(part)
        while view:
            remaining=deadline-time.monotonic()
            if remaining <= 0 or not select.select([], [stream], [], max(0,remaining))[1]:
                raise TimeoutError('Inference transport write deadline')
            try:
                count=os.write(stream.fileno(),view[:65536])
            except BlockingIOError:
                continue
            if count<=0:
                raise EOFError('Inference transport write closed')
            view=view[count:]


def validate_request(header, payload):
    if (header.get('model') not in ('fcos','centerpoint') or
            type(header.get('sequence')) is not int or header['sequence']<=0 or
            type(header.get('stamp_ns')) is not int or header['stamp_ns']<=0):
        raise ValueError('Invalid request identity')
    if header['model']=='fcos':
        width,height=header.get('width'),header.get('height')
        if type(width) is not int or type(height) is not int or not 64<=width<=1920 or not 64<=height<=1080:
            raise ValueError('Invalid image dimensions')
        rows=round(height*min(896/width,896/height))
        if header.get('rows')!=rows or len(payload)!=896*(rows+(rows+1)//2):
            raise ValueError('Invalid cropped NV12 payload')
    else:
        n=header.get('points')
        if type(n) is not int or not 1<=n<=300000 or len(payload)!=n*5*4:
            raise ValueError('Invalid point payload')


class Client:
    def __init__(self, remote=False, timeout=2.0):
        self.child=None; self.socket=None; self.timeout=timeout
        if remote:
            self.child=subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=4',
                'root@192.168.10.100','python3','-u',
                '/map/robot_j6m_optimized_20260905/bpu_perception_20260906/tools/stdio_relay.py'],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,bufsize=0)
            self.reader,self.writer=self.child.stdout,self.child.stdin
        else:
            self.socket=socket.create_connection(('127.0.0.1',PORT),timeout=timeout)
            self.socket.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.reader=self.writer=self.socket
        self.last_sequence=0

    def infer(self, header, payload):
        validate_request(header,payload)
        if header['sequence']<=self.last_sequence:
            raise ValueError('Non-increasing request sequence')
        send(self.writer,header,payload,self.timeout)
        result,extra=receive(self.reader,self.timeout)
        if (extra or any(result.get(k)!=header[k] for k in ('model','sequence','stamp_ns')) or
                result.get('motion_eligible') is not False):
            raise ValueError('Inference result identity mismatch')
        self.last_sequence=header['sequence']
        if not result.get('ok'):
            raise RuntimeError(result.get('error','Inference failed'))
        return result

    def close(self):
        if self.socket:
            self.socket.close(); self.socket=None
        if self.child:
            self.child.stdin.close()
            try:self.child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.child.terminate()
                try:self.child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.child.kill(); self.child.wait(timeout=2)
            self.child.stdout.close(); self.child=None
