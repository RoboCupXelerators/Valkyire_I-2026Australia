#!/usr/bin/env python3
"""
Test runner: run main.py for a short time under the project's venv and collect logs.
This script does NOT enable systemd; it just emulates how the service will run.
"""
import os
import time
import subprocess
import signal

PROJECT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
VENV_PY = os.path.join(PROJECT_DIR, 'venv', 'bin', 'python')
MAIN_PY = os.path.join(PROJECT_DIR, 'src', 'main.py')
LOG_DIR = os.path.join(PROJECT_DIR, 'log')

os.makedirs(LOG_DIR, exist_ok=True)

if not os.path.exists(VENV_PY):
    raise SystemExit(f"Virtualenv python not found at {VENV_PY}")
if not os.path.exists(MAIN_PY):
    raise SystemExit(f"main.py not found at {MAIN_PY}")

out_path = os.path.join(LOG_DIR, 'main_test.out')
err_path = os.path.join(LOG_DIR, 'main_test.err')

print('Starting main.py under venv for 6 seconds (PID will be printed)')
with open(out_path, 'ab') as out, open(err_path, 'ab') as err:
    p = subprocess.Popen([VENV_PY, '-u', MAIN_PY], stdout=out, stderr=err, cwd=os.path.join(PROJECT_DIR, 'src'), preexec_fn=os.setsid)
    print('PID:', p.pid)
    try:
        time.sleep(6)
        print('Sending SIGINT to process group')
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=5)
        print('Process exited with code', p.returncode)
    except Exception as e:
        print('Exception during test run:', e)
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

print('Test logs:')
print('--- tail of stdout ---')
with open(out_path, 'rb') as f:
    f.seek(0, os.SEEK_END)
    size = f.tell()
    f.seek(max(0, size-2000))
    print(f.read().decode(errors='ignore'))
print('--- tail of stderr ---')
with open(err_path, 'rb') as f:
    f.seek(0, os.SEEK_END)
    size = f.tell()
    f.seek(max(0, size-2000))
    print(f.read().decode(errors='ignore'))
