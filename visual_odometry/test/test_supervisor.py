"""Owned process-group rollover checks; no camera or GPU qualification."""
# Copyright (c) 2026 Paul N. Beuchat. MIT License.
import importlib.machinery
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

loader = importlib.machinery.SourceFileLoader('odometry_supervisor', str(Path(__file__).parents[1]/'scripts/odometry_supervisor'))
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)


class ProcessLifecycle(unittest.TestCase):
    def stop(self, child):
        child.stop(time.monotonic())
        deadline = time.monotonic()+7
        while not child.stopped(time.monotonic()) and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertIsNotNone(child.process.returncode)
        self.assertFalse(child.alive(), 'descendant survived tracker rollover')

    def test_launcher_exit_does_not_hide_descendant(self):
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory)/'ready'
            worker = 'import signal,time,pathlib; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); pathlib.Path('+repr(str(ready))+').touch(); time.sleep(30)'
            launcher = 'import subprocess,sys; subprocess.Popen([sys.executable,"-c",'+repr(worker)+'])'
            child = module.ChildGroup([sys.executable, '-c', launcher])
            try:
                deadline=time.monotonic()+3
                while not ready.exists() and time.monotonic()<deadline:
                    time.sleep(.02)
                self.assertTrue(ready.exists())
                child.process.wait(timeout=2)
                self.assertTrue(child.alive(), 'test did not retain a descendant')
                self.stop(child)
            finally:
                child.send(signal.SIGKILL)
                child.process.wait(timeout=2)

    def test_normal_shutdown(self):
        child = module.ChildGroup([sys.executable, '-c', 'import time; time.sleep(30)'])
        try:
            self.stop(child)
        finally:
            child.send(signal.SIGKILL)
            child.process.wait(timeout=2)

    def test_repeated_signals_do_not_abort_final_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ready, stopping, child_pid = (root/name for name in ('ready', 'stopping', 'child_pid'))
            worker = ('import signal,time,pathlib; '
                      'signal.signal(signal.SIGINT,lambda *_: pathlib.Path('+repr(str(stopping))+').touch()); '
                      'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                      'pathlib.Path('+repr(str(ready))+').touch(); time.sleep(30)')
            parent = ('import importlib.machinery,importlib.util,pathlib,sys,time; '
                      'loader=importlib.machinery.SourceFileLoader("supervisor",'+repr(loader.path)+'); '
                      'spec=importlib.util.spec_from_loader(loader.name,loader); '
                      'm=importlib.util.module_from_spec(spec); loader.exec_module(m); '
                      'child=m.ChildGroup([sys.executable,"-c",'+repr(worker)+']); '
                      'pathlib.Path('+repr(str(child_pid))+').write_text(str(child.pid)); '
                      '\nwhile not pathlib.Path('+repr(str(ready))+').exists(): time.sleep(.02)\n'
                      'm.finish_child(child); assert child.process.returncode is not None and not child.alive()')
            process = subprocess.Popen([sys.executable, '-c', parent])
            try:
                deadline = time.monotonic()+3
                while not stopping.exists() and time.monotonic()<deadline:
                    time.sleep(.02)
                self.assertTrue(stopping.exists(), 'cleanup never signalled the child')
                process.send_signal(signal.SIGINT)
                process.send_signal(signal.SIGTERM)
                self.assertEqual(process.wait(timeout=7), 0)
                self.assertFalse(Path('/proc', child_pid.read_text()).exists())
            finally:
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=2)
                if child_pid.exists():
                    try:
                        os.killpg(int(child_pid.read_text()), signal.SIGKILL)
                    except ProcessLookupError:
                        pass


if __name__ == '__main__':
    unittest.main()
