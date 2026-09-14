"""Result metadata only; does not authorize motion or change controller states."""
import copy
import uuid


class VisualSessionReceipt:
    def __init__(self):
        self.session_id = ""
        self.result = "IDLE"
        self.sequence = 0
        self.last_completion = {}

    def begin(self):
        self.session_id = uuid.uuid4().hex
        self.result = "RUNNING"

    def abort(self):
        if self.session_id:
            self.result = "ABORT"

    def disable(self):
        if self.session_id and self.result not in ("COMPLETE", "ABORT"):
            self.result = "CANCELLED"

    def complete(self, stopped=True):
        # Sensor-fenced cleaning result. A zero command alone is not stopped.
        if not self.session_id or self.result != "RUNNING":
            return
        self.sequence += 1
        self.result = "COMPLETE"
        self.last_completion = dict(session_id=self.session_id, sequence=self.sequence,
                                    result="COMPLETE", stopped=stopped)

    def status(self):
        return dict(session_id=self.session_id, result=self.result,
                    sequence=self.sequence, last_completion=copy.deepcopy(self.last_completion))
