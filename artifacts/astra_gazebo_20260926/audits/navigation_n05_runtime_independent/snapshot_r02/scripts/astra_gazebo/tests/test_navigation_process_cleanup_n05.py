"""Actual cleanup function, fake sockets/owned-process query; no signal sent."""
import json
from pathlib import Path
import sys
import types
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import navigation_process_cleanup_n05 as cleanup

@pytest.mark.parametrize('occupied',[None,14830,14831])
def test_exact_new_ports_are_required_before_receipt(tmp_path,monkeypatch,occupied):
    binds=[]
    class Socket:
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def setsockopt(self,*a):pass
        def bind(self,address):
            binds.append(address)
            if address[1]==occupied:raise OSError('occupied new port')
    monkeypatch.setattr(cleanup.socket,'socket',Socket)
    monkeypatch.setattr(cleanup,'owned_processes',lambda _:[])
    proc=types.SimpleNamespace(poll=lambda:0)
    if occupied is not None:
        with pytest.raises(OSError):cleanup.stop_group(proc,tmp_path)
        assert not (tmp_path/'cleanup.json').exists()
    else:
        cleanup.stop_group(proc,tmp_path)
        assert binds==[('127.0.0.1',14830),('127.0.0.1',14831)]
        report=json.loads((tmp_path/'cleanup.json').read_text())
        assert report['ports_14830_14831_released'] is True
        assert 'ports_14820_14821_released' not in report
