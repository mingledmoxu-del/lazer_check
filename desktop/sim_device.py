import threading, time
from PySide6.QtCore import QObject, Signal

class SimDevice(QObject):
    line = Signal(str)
    def __init__(self):
        super().__init__(); self.running = True; self.enabled = False; self.rpm = 0
        self._t = threading.Thread(target=self._loop, daemon=True); self._t.start()
    def send(self, text):
        p = text.strip().split(); rid = p[1] if len(p) > 1 else "0"; cmd = p[2] if len(p) > 2 else ""
        if cmd == "CHECK": self.line.emit(f"@R {rid} OK failed=0xFFFF actual=0 expected=0")
        elif cmd == "ENABLE": self.enabled=True; self.line.emit(f"@R {rid} OK enabled=1")
        elif cmd == "RUN":
            v = int(next(x.split('=')[1] for x in p if x.startswith('rpm='))); self.rpm=v
            self.line.emit(f"@R {rid} OK running=1 rpm={v}")
        elif cmd in ("STOP", "DISABLE"): self.rpm=0; self.enabled = cmd != "DISABLE"; self.line.emit(f"@R {rid} OK enabled={int(self.enabled)} rpm=0")
        elif cmd == "STATUS": self.line.emit(f"@R {rid} OK enabled={int(self.enabled)} rpm={self.rpm} alarm=0 age_ms=0")
        else: self.line.emit(f"@R {rid} REJECT code=BAD_COMMAND")
    def _loop(self):
        while self.running:
            time.sleep(.2)
            self.line.emit(f"@S 0 OK enabled={int(self.enabled)} rpm={self.rpm} alarm=0 age_ms=0")
    def close(self): self.running=False
