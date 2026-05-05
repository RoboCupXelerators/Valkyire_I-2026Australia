#!/usr/bin/env python3
"""Test MJPEG stream from Unity simulator."""
import urllib.request
import time

url = 'http://127.0.0.1:20801/stream?id=robot1_0'
print(f'Testing {url}...')

try:
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=60) as response:
        print(f'Status: {response.status}')
        print(f'Content-Type: {response.headers.get("Content-Type")}')
        
        # Read multiple frames to verify they are different
        frames = []
        frame_count = 0
        data = b''
        
        start_time = time.time()
        
        while frame_count < 10 and time.time() - start_time < 30:
            chunk = response.read(8192)
            if chunk:
                data += chunk
            
            # Find JPEG markers
            while True:
                jpeg_start = data.find(b'\xff\xd8\xff')
                if jpeg_start >= 0:
                    # Find JPEG end marker
                    jpeg_end = data.find(b'\xff\xd9', jpeg_start + 3)
                    if jpeg_end >= 0:
                        jpeg_end += 2  # Include end marker
                        frame_data = data[jpeg_start:jpeg_end]
                        frames.append(frame_data)
                        frame_count += 1
                        print(f'Frame {frame_count}: {len(frame_data)} bytes')
                        data = data[jpeg_end:]  # Remove processed frame
                    else:
                        break  # Need more data
                else:
                    break  # Need more data
        
        print(f'\nTotal frames received: {len(frames)}')
        
        # Check if frames are different (indicating live update)
        if len(frames) >= 2:
            same_count = 0
            for i in range(1, len(frames)):
                if frames[i] == frames[i-1]:
                    same_count += 1
            print(f'Frames identical: {same_count}/{len(frames)-1}')
            if same_count == len(frames) - 1:
                print('WARNING: All frames are identical - camera not updating!')
            else:
                print('SUCCESS: Frames are different - camera is updating!')
        
except Exception as e:
    print(f'Error: {type(e).__name__}: {e}')