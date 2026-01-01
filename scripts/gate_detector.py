import cv2
import numpy as np

class GateDetector:
    def __init__(self):
        # HSV Color Ranges
        # Adjust these based on actual underwater footage
        self.colors = {
            'red_lower1': np.array([0, 120, 50]),
            'red_upper1': np.array([8, 255, 255]),
            'red_lower2': np.array([172, 120, 50]),
            'red_upper2': np.array([180, 255, 255]),
            'green_lower': np.array([40, 50, 50]),
            'green_upper': np.array([90, 255, 255])
        }
        
        # Kernel for morphological operations
        # Use a vertical kernel to merge the dashed segments of the post
        # (width, height) -> tall and thin kernel
        self.merge_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 50))
        
    def detect(self, frame):
        output_data = {
            'gate_found': False,
            'center': None,  # (x, y) target center
            'red_post': None, # bbox (x, y, w, h)
            'green_post': None # bbox (x, y, w, h)
        }
        
        # 1. Preprocessing
        # CLAHE + Blur to enhance contrast underwater
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        enhanced = cv2.merge((cl,a,b))
        enhanced_bgr = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        
        hsv = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2HSV)
        
        # 2. Color Masking
        # Red Mask (handling wrap-around)
        mask_r1 = cv2.inRange(hsv, self.colors['red_lower1'], self.colors['red_upper1'])
        mask_r2 = cv2.inRange(hsv, self.colors['red_lower2'], self.colors['red_upper2'])
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)
        
        # Green Mask
        mask_green = cv2.inRange(hsv, self.colors['green_lower'], self.colors['green_upper'])
        
        # 3. Dilate untuk menggabungkan merah/hijau yang terputus-putus
        mask_red_merged = cv2.dilate(mask_red, self.merge_kernel, iterations=1)
        mask_green_merged = cv2.dilate(mask_green, self.merge_kernel, iterations=1)
        
        # 4. Find Best Posts
        red_post = self._find_largest_vertical_contour(mask_red_merged)
        green_post = self._find_largest_vertical_contour(mask_green_merged)
        
        # Simpan hasil deteksi
        if red_post: output_data['red_post'] = red_post
        if green_post: output_data['green_post'] = green_post
        
        # 5. Gate Logic
        # We need at least one post to estimate, but preferably both.
        if red_post and green_post:
            output_data['gate_found'] = True
            rx, ry, rw, rh = red_post
            gx, gy, gw, gh = green_post
            
            # Target center is midpoint between the two posts
            center_x = int((rx + rw/2 + gx + gw/2) / 2)
            center_y = int((ry + rh/2 + gy + gh/2) / 2)
            output_data['center'] = (center_x, center_y)
        
        return output_data, enhanced_bgr

    def _find_largest_vertical_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Filter huge noise or tiny speckles
            if area < 300: continue
                
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(h) / w
            if aspect_ratio > 1.5:
                if area > max_area:
                    max_area = area
                    best_cnt = (x, y, w, h)
                    
        return best_cnt

    # Kalau bagian kiri gate (merah) kelihatan lebih tinggi, bot di kiri
    # Kalau bagian kanan gate (hijau) kelihatan lebih tinggi, bot di kanan
    def check_gate_alignment(self, red_bbox, green_bbox, height_threshold=5):
        if red_bbox is None or green_bbox is None:
            return "unknown", 0.0

        # Extract heights (index 3 in x, y, w, h)
        h_left = red_bbox[3]
        h_right = green_bbox[3]
        
        # Positif: kiri lebih tinggi (Bot di kiri)
        # Negatif: kanan lebih tinggi (Bot di kanan)
        diff = h_left - h_right

        if abs(diff) < height_threshold: return "centered", diff
        elif h_left > h_right: return "move_right", diff
        else: return "move_left", diff

    def draw_results(self, frame, data):
        output = frame.copy()
        
        # Draw Red Post
        if data['red_post']:
            x, y, w, h = data['red_post']
            cv2.rectangle(output, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(output, "Red Post", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
        # Draw Green Post
        if data['green_post']:
            x, y, w, h = data['green_post']
            cv2.rectangle(output, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(output, "Green Post", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        # Draw Gate Center
        if data['gate_found']:
            cx, cy = data['center']
            # Draw crosshair
            cv2.circle(output, (cx, cy), 10, (255, 255, 0), -1)
            cv2.line(output, (data['red_post'][0], cy), (data['green_post'][0], cy), (255, 255, 0), 2)
            cv2.putText(output, "GATE TARGET", (cx-40, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
        return output