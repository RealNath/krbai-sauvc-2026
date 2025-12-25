import cv2
import numpy as np

class FlareDetector:
    def __init__(self):
        # HSV Color Range ()
        # Note: Red wraps around 180 in HSV
        # TODO: HSV belum tentu pas
        self.colors = {
            'red_lower': ([0, 50, 50], [7, 255, 255]),
            'red_upper': ([170, 50, 50], [180, 255, 255]),
            'blue': ([100, 50, 50], [130, 255, 255]),
            'yellow': ([20, 50, 50], [40, 255, 255]),
            'orange': ([8, 50, 50], [25, 255, 255]) # Adjusted for #b44d25 (Hue ~8)
        }
        
        # Detection parameters
        self.min_aspect_ratio = 3.0 # tinggi / lebar (Flare vertikal)
        self.min_area = 100 # abaikan noise
        
    def detect(self, frame):
        """
        Proses:
        1. Preprocess (Blur -> Edge Detection)
        2. Cari contour (kandidat objek yang vertikal)
        3. Extract warna (kalau cukup besar)
        """
        results = []
        
        # 1. Preprocessing
        # Gambar bawah air cenderung dominan biru/hijau.
        # CLAHE (Contrast Limited Adaptive Histogram Equalization) membantu mencari "L" dari LAB.
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl,a,b))
        enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        
        gray = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # 2. Cari Contour Vertikal
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
                
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(h) / w
            
            # Cek apakah merupakan object vertikal
            if aspect_ratio > self.min_aspect_ratio:
                # 3. Cek Warna (Heuristik: Rata2 warna dari area tengah)
                # Cek warna kalau objek "cukup besar" (kamera mendekat)
                # Kalau tdk, kita kembalikan 'unknown'
                color_label = 'unknown'
                
                # Ambil ROI untuk deteksi warna
                roi = frame[y:y+h, x:x+w]
                if roi.size > 0:
                    color_label = self._identify_color(roi)
                
                results.append({
                    'bbox': (x, y, w, h),
                    'color': color_label,
                    'aspect_ratio': aspect_ratio,
                    'area': area
                })
                
        return results, enhanced

    # TODO: Belum akurat kayaknya
    def _identify_color(self, roi):
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        height, width, _ = hsv_roi.shape
        
        # Pakai area tengah (misal 50% lebar/tinggi)
        # untuk memastikan fokus ke objek, bukan background di sudut bounding box
        center_h, center_w = height // 2, width // 2
        h_start = max(0, center_h - height // 4)
        h_end = min(height, center_h + height // 4)
        w_start = max(0, center_w - width // 4)
        w_end = min(width, center_w + width // 4)
        
        target_region = hsv_roi[h_start:h_end, w_start:w_end]
        
        if target_region.size == 0:
            return 'unknown'

        max_count = 0
        detected_color = 'unknown'
        total_pixels = target_region.shape[0] * target_region.shape[1]
        
        for color_name, ranges in self.colors.items():
            if 'red' in color_name:
                continue 
                
            lower = np.array(ranges[0])
            upper = np.array(ranges[1])
            mask = cv2.inRange(target_region, lower, upper)
            count = cv2.countNonZero(mask)
            
            if count > max_count:
                max_count = count
                detected_color = color_name
        
        # Handle merah (wrap around)
        lower1 = np.array(self.colors['red_lower'][0])
        upper1 = np.array(self.colors['red_lower'][1])
        lower2 = np.array(self.colors['red_upper'][0])
        upper2 = np.array(self.colors['red_upper'][1])
        
        mask1 = cv2.inRange(target_region, lower1, upper1)
        mask2 = cv2.inRange(target_region, lower2, upper2)
        red_count = cv2.countNonZero(mask1) + cv2.countNonZero(mask2)
        
        if red_count > max_count:
            max_count = red_count
            detected_color = 'red'
            
        # Threshold: warna harus muncul di minimal 10% pixel tengah
        if max_count < (0.1 * total_pixels):
            return 'unknown'
            
        return detected_color

    def draw_results(self, frame, results):
        """
        Fungsi untuk buat bounding box dan label pada frame
        """
        output_frame = frame.copy()
        
        for res in results:
            x, y, w, h = res['bbox']
            color = res['color']
            aspect_ratio = res['aspect_ratio']
            
            # Pilih warna untuk box (hijau: dikenal; merah: tidak dikenal)
            box_color = (0, 255, 0) if color != 'unknown' else (0, 0, 255)
            
            # Buat bounding box
            cv2.rectangle(output_frame, (x, y), (x+w, y+h), box_color, 2)
            
            # Buat label
            label = f"{color} (AR:{aspect_ratio:.1f})"
            cv2.putText(output_frame, label, (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                       
        return output_frame
