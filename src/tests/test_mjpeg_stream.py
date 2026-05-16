"""Test MJPEG stream connection."""

import socket
import time
import requests

# Test raw TCP connection with HTTP/1.0
print("=== Testing raw TCP with HTTP/1.0 ===")
host = '127.0.0.1'
port = 20801

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    print(f'Connecting to {host}:{port}...')
    sock.connect((host, port))
    print('Connected!')
    
    # Send HTTP/1.0 request for stream
    request = b'GET /stream?id=robot1_0 HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n'
    print(f'Sending request...')
    sock.sendall(request)
    
    # Wait for response
    print('Waiting for response...')
    sock.settimeout(10)
    response = b''
    start_time = time.time()
    while time.time() - start_time < 10:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            response += chunk
            if len(response) > 500:
                print(f'Received {len(chunk)} bytes, total: {len(response)}')
                break
        except socket.timeout:
            print(f'Socket timeout after {time.time() - start_time:.1f}s')
            break
    
    print(f'Total received: {len(response)} bytes in {time.time() - start_time:.1f}s')
    if response:
        print(f'Response headers:\n{response.decode("utf-8", errors="ignore")[:300]}')
    else:
        print('No response received!')
    sock.close()
except Exception as e:
    print(f'TCP Error: {e}')

print()

# Test with requests library for stream
print("=== Testing with requests library ===")
stream_url = 'http://127.0.0.1:20801/stream?id=robot1_0'
print(f'Testing stream endpoint: {stream_url}')

try:
    r = requests.get(stream_url, stream=True, timeout=10)
    print(f'Stream response status: {r.status_code}')
    print(f'Content-Type: {r.headers.get("Content-Type")}')
    
    # Read some data
    data = b''
    start_time = time.time()
    for chunk in r.iter_content(chunk_size=1024):
        data += chunk
        if len(data) > 1000:
            break
        if time.time() - start_time > 5:
            break
    
    print(f'Received {len(data)} bytes')
    if data:
        print(f'First 300 bytes:\n{data[:300]}')
except Exception as e:
    print(f'Stream error: {e}')