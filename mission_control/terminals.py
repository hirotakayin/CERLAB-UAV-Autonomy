"""Debug terminals for the page: a real bash in a pseudo-terminal, with the same ROS setup and environment as the tab's
processes (sim: local master; drone: the drone's master, or an SSH shell on the drone). The page streams output (base64
over server-sent events) and sends keystrokes back. Closed with the page's x, when bash exits, or when the server stops.
"""
import base64
import fcntl
import os
import pty
import select
import signal
import struct
import termios
import threading
import time
import uuid


class Terminal:
    def __init__(self, argv, env, title, cwd=None):
        self.id, self.title, self.started = uuid.uuid4().hex[:8], title, time.time()
        self.pid, self.fd = pty.fork()
        if self.pid == 0:                                   # child: become the shell
            try:
                if cwd: os.chdir(cwd)
                os.execvpe(argv[0], argv, env)
            finally:
                os._exit(127)
        self.buf = bytearray()                              # output not yet sent, capped
        self.history = bytearray()                          # last 64 KB, replayed when the page reconnects
        self.cv = threading.Condition()
        self.alive = True
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while True:
            try:
                r, _, _ = select.select([self.fd], [], [], 1.0)
                if not r: continue
                data = os.read(self.fd, 65536)
            except OSError:
                data = b''
            if not data:
                break
            with self.cv:
                self.buf += data; self.history += data
                del self.history[:-65536]
                del self.buf[:-262144]
                self.cv.notify_all()
        with self.cv:
            self.alive = False; self.cv.notify_all()
        try: os.waitpid(self.pid, 0)
        except ChildProcessError: pass

    def write(self, data):
        os.write(self.fd, data)

    def resize(self, cols, rows):
        fcntl.ioctl(self.fd, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols, 0, 0))

    def stream(self, send):
        """Calls send(text) with base64 chunks until the shell exits or send raises."""
        with self.cv:
            pending = bytes(self.history); self.buf.clear()
        if pending: send(base64.b64encode(pending).decode())
        while True:
            with self.cv:
                if not self.buf and self.alive: self.cv.wait(15)
                data, alive = bytes(self.buf), self.alive
                self.buf.clear()
            send(base64.b64encode(data).decode() if data else '')   # empty = keep-alive
            if not alive and not data:
                return

    def close(self):
        try: os.killpg(os.getpgid(self.pid), signal.SIGHUP)
        except (ProcessLookupError, PermissionError): pass
        try: os.close(self.fd)
        except OSError: pass


class Terminals:
    def __init__(self):
        self.terms = {}

    def open(self, argv, env, title, cwd=None):
        t = Terminal(argv, env, title, cwd)
        self.terms[t.id] = t
        return t

    def get(self, tid):
        return self.terms.get(tid)

    def close(self, tid):
        t = self.terms.pop(tid, None)
        if t: t.close()

    def list(self):
        return [{'id': t.id, 'title': t.title, 'alive': t.alive} for t in self.terms.values()]

    def close_all(self):
        for tid in list(self.terms): self.close(tid)
