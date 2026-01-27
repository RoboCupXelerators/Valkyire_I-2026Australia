import subprocess
import time
import signal
import os

def start_servers():
    # Start vision server
    vision_process = subprocess.Popen(["python", "src/vision_server.py"], cwd=os.getcwd())
    print("Started vision server")

    # Start sensor server
    sensor_process = subprocess.Popen(["python", "src/sensor_server.py"], cwd=os.getcwd())
    print("Started sensor server")

    try:
        # Keep running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping servers...")
        vision_process.terminate()
        sensor_process.terminate()
        vision_process.wait()
        sensor_process.wait()
        print("Servers stopped")

if __name__ == "__main__":
    start_servers()