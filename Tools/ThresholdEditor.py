import cv2
import numpy as np

def nothing(x):
    pass

# Create a window
cv2.namedWindow('HSV Threshold Editor')

# Create trackbars for color change
# Hue is from 0-179 in OpenCV
cv2.createTrackbar('H_min', 'HSV Threshold Editor', 0, 179, nothing)
cv2.createTrackbar('S_min', 'HSV Threshold Editor', 0, 255, nothing)
cv2.createTrackbar('V_min', 'HSV Threshold Editor', 0, 255, nothing)
cv2.createTrackbar('H_max', 'HSV Threshold Editor', 179, 179, nothing)
cv2.createTrackbar('S_max', 'HSV Threshold Editor', 255, 255, nothing)
cv2.createTrackbar('V_max', 'HSV Threshold Editor', 255, 255, nothing)

# Create switch for ON/OFF functionality
cv2.createTrackbar('Use Camera', 'HSV Threshold Editor', 0, 1, nothing)

# Optional: Set default value for HSV Max
cv2.setTrackbarPos('H_min', 'HSV Threshold Editor', 10)
cv2.setTrackbarPos('H_max', 'HSV Threshold Editor', 15)
cv2.setTrackbarPos('S_min', 'HSV Threshold Editor', 128)
cv2.setTrackbarPos('S_max', 'HSV Threshold Editor', 255)
cv2.setTrackbarPos('V_min', 'HSV Threshold Editor', 150)
cv2.setTrackbarPos('V_max', 'HSV Threshold Editor', 255)
cv2.setTrackbarPos('Use Camera', 'HSV Threshold Editor', 1)

# Load a default image
# default_image = np.ones((300, 300, 3), dtype=np.uint8) * 100
# You can replace this with your own image: default_image = cv2.imread('your_image.jpg')

# Initialize webcam (if available)
camera = cv2.VideoCapture(0)

print("Press 'ESC' to exit")
print("Press 's' to save current HSV values to file")

while True:
    # Get current positions of trackbars
    h_min = cv2.getTrackbarPos('H_min', 'HSV Threshold Editor')
    s_min = cv2.getTrackbarPos('S_min', 'HSV Threshold Editor')
    v_min = cv2.getTrackbarPos('V_min', 'HSV Threshold Editor')
    h_max = cv2.getTrackbarPos('H_max', 'HSV Threshold Editor')
    s_max = cv2.getTrackbarPos('S_max', 'HSV Threshold Editor')
    v_max = cv2.getTrackbarPos('V_max', 'HSV Threshold Editor')
    use_camera = cv2.getTrackbarPos('Use Camera', 'HSV Threshold Editor')
    
    if use_camera == 1:
        ret, frame = camera.read()
        if not ret:
            print("Failed to grab frame from camera")
            use_camera = 0
            cv2.setTrackbarPos('Use Camera', 'HSV Threshold Editor', 0)
            continue
    else:
        frame = default_image.copy()
    
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range of color in HSV
    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])
    
    # Threshold the HSV image to get only the specified colors
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Bitwise-AND mask and original image
    result = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Stack images horizontally for display
    display = np.hstack((frame, result))
    
    # If the display is too big, resize it
    if display.shape[1] > 1200:
        display = cv2.resize(display, (1200, int(1200 * display.shape[0] / display.shape[1])))
    
    # Display current HSV range
    hsv_info = f"H: {h_min}-{h_max}, S: {s_min}-{s_max}, V: {v_min}-{v_max}"
    cv2.putText(display, hsv_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Show the result
    cv2.imshow('HSV Threshold Editor', display)
    
    # Handle key presses
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC key to exit
        break
    elif key == ord('s'):  # 's' key to save values
        with open('hsv_values.txt', 'w') as f:
            f.write(f"HSV Min: [{h_min}, {s_min}, {v_min}]\n")
            f.write(f"HSV Max: [{h_max}, {s_max}, {v_max}]\n")
        print(f"Saved HSV values to hsv_values.txt")
        print(f"HSV Min: [{h_min}, {s_min}, {v_min}]")
        print(f"HSV Max: [{h_max}, {s_max}, {v_max}]")

# Release resources
if camera.isOpened():
    camera.release()
cv2.destroyAllWindows()
