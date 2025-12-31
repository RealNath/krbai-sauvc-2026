#!/usr/bin/env python3
import cv2
import sys
import os

# Path setup similar to your existing code
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from gate_detector import GateDetector

def main():
    detector = GateDetector()
    cap = cv2.VideoCapture(0) # Change to video filename if testing locally
    
    while True:
        ret, frame = cap.read()
        if not ret: break
            
        data, processed_img = detector.detect(frame)
        output_frame = detector.draw_results(frame, data)
        
        cv2.imshow('Gate Detection', output_frame)
        # Uncomment below to see how the mask looks (debug)
        # cv2.imshow('Processed', processed_img)

        direction, error = check_gate_alignment(results['red_post'], results['green_post'])
        print(f"Status: {direction} (Diff: {error})")
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()