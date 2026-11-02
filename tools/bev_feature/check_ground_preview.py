#!/usr/bin/env python3
"""Isolated HTTP/Qt-packet regression with recorded images and explicit test clock."""
import argparse
import copy
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import urllib.request

import cv2
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'distribution/j6m_bpu_navigation'))
from surround.ground_protocol import encode_view_frame


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    saved=ROOT/'runtime/surround_recalibrated_20261101/bev'
    original=json.loads((saved/'bev.json').read_text())
    images={n:cv2.imread(str(saved/(n+'_upright.png'))) for n in original['sources']}
    _,jpeg=cv2.imencode('.jpg',cv2.imread(str(saved/'bev.png')))
    class Fixture(BaseHTTPRequestHandler):
        sequence=0
        def log_message(self,*_):pass
        def do_GET(self):
            Fixture.sequence+=1
            meta=copy.deepcopy(original)
            meta.update(fusion_session='OFFLINE_HTTP_REGRESSION',fusion_sequence=Fixture.sequence,
                produced_epoch_s=time.time(),protocol='conbine.ground_bev.frame.v1',image_width=600,image_height=600,
                camera_image_sizes={n:list(im.shape[1::-1]) for n,im in images.items()},
                offline_replay=True,synthetic_delivery_clock=True,
                original_capture_metadata=original['sources'],source_selection='recorded images with synthetic regression clock')
            for source in meta['sources'].values():
                source.update(received_epoch_ns=time.time_ns(),source_age_s=0.,producer_cache_age_s=0.)
            header=json.dumps(meta).encode()
            bg=struct.pack('!4sII',b'BG01',len(header),len(jpeg))+header+jpeg.tobytes()
            packet=encode_view_frame(bg,images)
            self.send_response(200);self.send_header('Content-Length',str(len(packet)));self.end_headers()
            try:self.wfile.write(packet)
            except BrokenPipeError:pass
    server=ThreadingHTTPServer(('127.0.0.1',18893),Fixture)
    server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    process=None
    with (args.output/'server.log').open('w') as log:
        try:
            process=subprocess.Popen(['/usr/bin/python3','-s',str(ROOT/'scripts/bev_yolo_server.py'),
                '--backend','bpu','--input','http://127.0.0.1:18893/views','--port','18894',
                '--output',str(args.output/'frames')],stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                if process.poll() is not None:raise RuntimeError('Preview server exited')
                try:
                    with urllib.request.urlopen('http://127.0.0.1:18894/frame',timeout=2) as response:packet=response.read()
                    magic,nh,nr,na=struct.unpack('!4sIII',packet[:16])
                    assert magic==b'BV01' and len(packet)==16+nh+nr+na
                    meta=json.loads(packet[16:16+nh])
                    assert meta['backend']=='BPU' and meta['inference_image_space']=='original_upright_cameras'
                    assert meta['source_id']==meta['image_source_id']==meta['result_source_id']
                    assert meta['source_metadata']['offline_replay']
                    assert len(meta['camera_detections'])==4
                    with urllib.request.urlopen('http://127.0.0.1:18894/detections',timeout=2) as response:observations=json.load(response)
                    assert observations['ground_frame']==original['frame_id']
                    assert observations['position_method']=='ground_contact_ray_plane'
                    (args.output/'preview_packet.bin').write_bytes(packet)
                    (args.output/'result.json').write_text(json.dumps(dict(passed=True,backend='BPU',camera_count=4,
                        source_id=meta['source_id'],pipeline_ms=meta['inference_ms'],offline_replay=True,
                        observed_objects=len(observations['ground_objects']),live_camera_test=False),indent=2)+'\n')
                    print('BPU original views -> geometry -> BEV packet/observations HTTP regression passed')
                    break
                except (OSError,ValueError):time.sleep(.2)
            else:raise TimeoutError('No complete preview result')
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=12)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            server.shutdown();server.server_close()


if __name__=='__main__':main()
