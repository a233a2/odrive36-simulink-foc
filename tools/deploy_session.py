"""Run a desktop deployment off the Tk event loop and retain its log."""
from contextlib import contextmanager
import ctypes
import locale
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import threading
import time
import traceback

from deploy_foc import deploy


class BuildCancelled(RuntimeError):
    pass


@contextmanager
def system_dll_search():
    # PyInstaller adds its extraction directory to the DLL search path.
    # MATLAB, Arm GCC and OpenOCD must load their own dependencies.
    if os.name != 'nt' or not getattr(sys, 'frozen', False):
        yield
        return
    set_directory = ctypes.windll.kernel32.SetDllDirectoryW
    set_directory.argtypes = [ctypes.c_wchar_p]
    set_directory(None)
    try:
        yield
    finally:
        set_directory(sys._MEIPASS)


def external_process(command, *, hidden=True, **kwargs):
    if os.name == 'nt' and hidden:
        kwargs['creationflags'] = subprocess.CREATE_NO_WINDOW
    with system_dll_search():
        return subprocess.Popen(command, **kwargs)


def decode_output(raw):
    if raw.startswith((b'\xff\xfe', b'\xfe\xff')):
        return raw.decode('utf-16', errors='replace')
    try:
        return raw.decode('utf-8-sig')
    except UnicodeDecodeError:
        return raw.decode(locale.getpreferredencoding(False), errors='replace')


def terminate_process_tree(process):
    if process.poll() is not None:
        return
    if os.name == 'nt':
        system_root = Path(os.environ.get('SystemRoot', r'C:\Windows'))
        killer = system_root / 'System32/taskkill.exe'
        try:
            with external_process([str(killer), '/PID', str(process.pid), '/T', '/F'],
                                  stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL) as command:
                command.wait(timeout=8)
        except (OSError, subprocess.TimeoutExpired):
            pass
    if process.poll() is None:
        process.kill()
    process.wait()


class BuildSession:
    def __init__(self, emit):
        self.emit = emit
        self.folder = None
        self.stop = threading.Event()
        self.phase_name = ''
        self.phase_lock = threading.Lock()
        self.log_error = False

    def request_cancel(self):
        with self.phase_lock:
            if self.phase_name in ('install', 'flash'):
                return False
            self.stop.set()
            return True

    def check_cancelled(self):
        if self.stop.is_set():
            raise BuildCancelled('构建已取消。')

    def phase(self, name, folder):
        self.folder = folder
        with self.phase_lock:
            self.check_cancelled()
            self.phase_name = name
        self.emit('phase', name)

    def report(self, message):
        message = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', str(message)).replace('\b', '')
        if not message.strip():
            return
        self.emit('log', message)
        if self.folder is not None and not self.log_error:
            try:
                with (self.folder / 'deploy.log').open('a', encoding='utf-8') as log:
                    log.write(message.rstrip() + '\n')
            except OSError as error:
                self.log_error = True
                self.emit('log', f'无法保存日志：{error}')

    def run(self, command, *, cwd=None, env=None, check=False, timeout=None,
            capture_output=False, text=False):
        self.check_cancelled()
        unfinished_error = None
        # Using a file avoids inheriting a pipe that might stay open in a
        # helper process after the vendor launcher exits.
        with tempfile.TemporaryFile() as buffer:
            process = external_process(command, cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                                       stdout=buffer, stderr=subprocess.STDOUT)
            started = time.monotonic()
            try:
                while process.poll() is None:
                    self.check_cancelled()
                    if timeout is not None and time.monotonic() - started >= timeout:
                        raise subprocess.TimeoutExpired(command, timeout)
                    self.stop.wait(0.12)
            except (BuildCancelled, subprocess.TimeoutExpired) as error:
                terminate_process_tree(process)
                unfinished_error = error
            buffer.seek(0)
            raw = buffer.read()
        if not capture_output:
            self.report(decode_output(raw))
        if unfinished_error is not None:
            raise unfinished_error
        self.check_cancelled()
        output = decode_output(raw) if text else raw
        result = subprocess.CompletedProcess(command, process.returncode, output)
        if check:
            result.check_returncode()
        return result

    def execute(self, options):
        try:
            stage = deploy(options, report=self.report, run=self.run, on_phase=self.phase)
        except BuildCancelled as error:
            self.report(str(error))
            self.emit('finish', ('cancelled', self.folder, str(error)))
        except Exception as error:
            if isinstance(error, subprocess.CalledProcessError):
                tool = Path(error.cmd[0]).name
                message = f'{tool} 退出状态 {error.returncode}。请查看构建日志。'
            else:
                message = str(error)
            self.report(message)
            if self.folder is not None:
                try:
                    (self.folder / 'error.log').write_text(traceback.format_exc(), encoding='utf-8')
                except OSError:
                    pass
            self.emit('finish', ('error', self.folder, message))
        else:
            self.emit('finish', ('ok', stage, ''))
