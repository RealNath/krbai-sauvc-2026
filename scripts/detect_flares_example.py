#!/usr/bin/env python3
import cv2
import sys
import os
import numpy as np

# Add the ROS package to path so we can import the library without sourcing ROS
# Assumes this script is run from the 'scripts' folder or root
current_dir = os.path.dirname(os.path.abspath(__file__))
ros_pkg_path = os.path.join(current_dir, '../ros2_ws/src/eggplant')
sys.path.append(ros_pkg_path)

try:
    from scripts.flare_detector import FlareDetector
except ImportError:
    print("Could not import vision. Make sure the path is correct.")
    sys.exit(1)

def main():
    # Initialize detector
    detector = FlareDetector()
    
    # Open webcam (0) or video file
    cap = cv2.VideoCapture('/path/to/video.mp4')
    
    if not cap.isOpened():
        print("Tidak bisa load video")
        return

    print("Tekan 'q' untuk keluar")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Run detection
        results, processed_frame = detector.detect(frame)

        # Draw the rectangles and show result
        output_frame = detector.draw_results(frame, results)
        cv2.imshow('Detections', output_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
