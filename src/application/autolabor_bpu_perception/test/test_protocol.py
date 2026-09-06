import socket
import threading
import time
import unittest
from autolabor_bpu_perception.protocol import send,receive,validate_request,PREFIX,MAGIC,MAX_PAYLOAD


class ProtocolTest(unittest.TestCase):
    def setUp(self):self.a,self.b=socket.socketpair()
    def tearDown(self):self.a.close();self.b.close()

    def test_binary_roundtrip(self):
        send(self.a,dict(model='centerpoint',sequence=1),b'\x00\xff')
        self.assertEqual(receive(self.b),(dict(model='centerpoint',sequence=1),b'\x00\xff'))

    def test_unread_large_write_has_real_deadline(self):
        self.a.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,4096)
        start=time.monotonic()
        with self.assertRaises(TimeoutError):send(self.a,{},b'x'*MAX_PAYLOAD,timeout=.03)
        self.assertLess(time.monotonic()-start,.5)

    def test_header_claim_cannot_allocate_unbounded_payload(self):
        self.a.sendall(PREFIX.pack(MAGIC,2,MAX_PAYLOAD+1))
        with self.assertRaises(ValueError):receive(self.b)

    def test_truncated_message_expires(self):
        self.a.sendall(PREFIX.pack(MAGIC,10,0))
        with self.assertRaises(TimeoutError):receive(self.b,timeout=.02)

    def test_request_rejects_boolean_sequence_and_wrong_tensor(self):
        for header,payload in [(dict(model='centerpoint',sequence=True,stamp_ns=1,points=1),bytes(20)),
                               (dict(model='centerpoint',sequence=1,stamp_ns=1,points=2),bytes(20))]:
            with self.assertRaises(ValueError):validate_request(header,payload)

    def test_nonfinite_json_is_rejected(self):
        raw=b'{"x":NaN}';self.a.sendall(PREFIX.pack(MAGIC,len(raw),0)+raw)
        with self.assertRaises(ValueError):receive(self.b)


if __name__=='__main__':unittest.main()
