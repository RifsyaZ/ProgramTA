import cv2
import numpy as np
import time
from collections import deque

class OptimizedVisualOdometry:
    def __init__(self, camera_index=1):
        self.camera_index = camera_index
        self.cap = None
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.prev_gray = None
        self.prev_time = None
        
        # History untuk smoothing
        self.pos_history_x = deque(maxlen=5)
        self.pos_history_y = deque(maxlen=5)
        self.dx_history = deque(maxlen=10)
        self.dy_history = deque(maxlen=10)
        
        # Statistik
        self.total_frames = 0
        self.fps_history = deque(maxlen=30)
        
        # Kalibrasi
        self.pixel_to_mm = 0.5  # Akan dihitung otomatis jika ada height
        
        # Parameter untuk deteksi
        self.texture_threshold = 50  # Lebih rendah = lebih sensitif
        self.min_movement = 0.1     # Minimum pergerakan untuk update
        
        # Untuk tracking performa
        self.no_movement_count = 0
        self.low_texture_count = 0
        
    def connect(self):
        """Konek ke kamera dengan setting optimal"""
        print(f"\nMengaktifkan kamera index {self.camera_index}...")
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            print(f"Error: Tidak bisa membuka kamera {self.camera_index}")
            return False
        
        # Set resolusi - lebih kecil untuk performa lebih baik
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)   # Turunkan resolusi
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        # Set parameter kamera untuk meningkatkan kontras
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 0.8)
        self.cap.set(cv2.CAP_PROP_SATURATION, 0.7)
        
        # Test frame
        ret, frame = self.cap.read()
        if ret:
            h, w = frame.shape[:2]
            print(f"Berhasil! Resolusi: {w}x{h}")
            return True
        else:
            print("Error: Kamera tidak mengirim frame")
            return False
    
    def _enhance_contrast(self, img):
        """Tingkatkan kontras gambar untuk deteksi lebih baik"""
        # CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(img)
        return enhanced
    
    def _compute_texture_score(self, img):
        """Hitung skor tekstur dengan metode lebih sensitif"""
        # Gunakan multiple metode untuk deteksi tekstur
        # 1. Laplacian variance
        lap = cv2.Laplacian(img, cv2.CV_64F)
        lap_variance = np.var(lap)
        
        # 2. Gradient magnitude
        gx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)
        grad_magnitude = np.mean(np.sqrt(gx**2 + gy**2))
        
        # Kombinasi
        texture_score = (lap_variance / 100) + (grad_magnitude / 10)
        
        return texture_score
    
    def _phase_correlate(self, img1, img2):
        """Phase correlation dengan optimasi"""
        h, w = img1.shape
        
        # Gunakan ukuran FFT yang lebih kecil untuk kecepatan
        window = np.outer(np.hanning(h), np.hanning(w))
        
        img1_fft = np.fft.fft2(img1.astype(np.float32) * window)
        img2_fft = np.fft.fft2(img2.astype(np.float32) * window)
        
        eps = 1e-8
        cross_power = (img1_fft * np.conj(img2_fft)) / (np.abs(img1_fft * np.conj(img2_fft)) + eps)
        correlation = np.fft.ifft2(cross_power).real
        
        h, w = correlation.shape
        y, x = np.unravel_index(np.argmax(correlation), correlation.shape)
        
        if x > w // 2:
            x = x - w
        if y > h // 2:
            y = y - h
            
        return float(x), float(y)
    
    def _optical_flow(self, img1, img2):
        """Optical flow dengan parameter lebih sensitif"""
        flow = cv2.calcOpticalFlowFarneback(
            img1, img2, None,
            pyr_scale=0.5,
            levels=3,
            winsize=15,
            iterations=3,
            poly_n=5,
            poly_sigma=1.2,
            flags=0
        )
        
        h, w = flow.shape[:2]
        margin_h = int(h * 0.2)
        margin_w = int(w * 0.2)
        
        flow_center = flow[margin_h:h-margin_h, margin_w:w-margin_w]
        
        # Gunakan median untuk robust
        dx = np.median(flow_center[:, :, 0])
        dy = np.median(flow_center[:, :, 1])
        
        # Filter noise: jika pergerakan terlalu kecil, anggap diam
        if abs(dx) < self.min_movement:
            dx = 0
        if abs(dy) < self.min_movement:
            dy = 0
        
        return float(dx), float(dy)
    
    def process(self, frame):
        """Proses frame untuk mendapatkan posisi"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Enhance kontras
        gray = self._enhance_contrast(gray)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_time = time.time()
            return None
        
        current_time = time.time()
        dt = current_time - self.prev_time
        
        # Hitung skor tekstur
        texture_score = self._compute_texture_score(gray)
        
        # Logging untuk debugging
        if texture_score < self.texture_threshold:
            self.low_texture_count += 1
        
        # Pilih metode berdasarkan tekstur
        if texture_score < self.texture_threshold:
            dx, dy = self._phase_correlate(self.prev_gray, gray)
            method = "Phase Correlation"
        else:
            dx, dy = self._optical_flow(self.prev_gray, gray)
            method = "Optical Flow"
        
        # Cek apakah ada pergerakan
        if abs(dx) < self.min_movement and abs(dy) < self.min_movement:
            self.no_movement_count += 1
        else:
            self.no_movement_count = 0
        
        # Smoothing
        self.dx_history.append(dx)
        self.dy_history.append(dy)
        
        dx_smooth = np.median(self.dx_history)
        dy_smooth = np.median(self.dy_history)
        
        # Update posisi
        self.pos_x += dx_smooth
        self.pos_y += dy_smooth
        
        # Smoothing posisi
        self.pos_history_x.append(self.pos_x)
        self.pos_history_y.append(self.pos_y)
        
        pos_x_smooth = np.mean(self.pos_history_x)
        pos_y_smooth = np.mean(self.pos_history_y)
        
        # FPS
        fps = 1.0 / dt if dt > 0 else 0
        self.fps_history.append(fps)
        avg_fps = np.mean(self.fps_history)
        
        self.total_frames += 1
        self.prev_gray = gray
        self.prev_time = current_time
        
        return {
            'pos_x': pos_x_smooth,
            'pos_y': pos_y_smooth,
            'pos_x_mm': pos_x_smooth * self.pixel_to_mm,
            'pos_y_mm': pos_y_smooth * self.pixel_to_mm,
            'dx': dx_smooth,
            'dy': dy_smooth,
            'method': method,
            'texture': texture_score,
            'fps': avg_fps,
            'frame': self.total_frames,
            'no_movement': self.no_movement_count
        }
    
    def get_frame(self):
        if self.cap:
            ret, frame = self.cap.read()
            if ret:
                return frame
        return None
    
    def reset(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.prev_gray = None
        self.pos_history_x.clear()
        self.pos_history_y.clear()
        self.dx_history.clear()
        self.dy_history.clear()
        self.total_frames = 0
        self.no_movement_count = 0
        self.low_texture_count = 0
    
    def release(self):
        if self.cap:
            self.cap.release()
    
    def get_stats(self):
        return {
            'low_texture_percent': (self.low_texture_count / max(self.total_frames, 1)) * 100,
            'no_movement_percent': (self.no_movement_count / max(self.total_frames, 1)) * 100
        }

def draw_info(frame, result, stats):
    """Gambar informasi dengan lebih detail"""
    h, w = frame.shape[:2]
    
    # Panel utama
    overlay = frame.copy()
    cv2.rectangle(overlay, (5, 5), (360, 210), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    
    # Title
    cv2.putText(frame, "VISUAL ODOMETRY (Optimized)", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    # Posisi
    cv2.putText(frame, f"X: {result['pos_x_mm']:.1f} mm", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"Y: {result['pos_y_mm']:.1f} mm", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Movement
    cv2.putText(frame, f"dX: {result['dx']:.2f} px", (10, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
    cv2.putText(frame, f"dY: {result['dy']:.2f} px", (10, 135),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
    
    # Metode dan tekstur
    method_color = (0, 165, 255) if "Phase" in result['method'] else (0, 255, 0)
    cv2.putText(frame, f"Method: {result['method']}", (10, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, method_color, 1)
    cv2.putText(frame, f"Texture: {result['texture']:.1f}", (10, 180),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    
    # FPS dan frame
    cv2.putText(frame, f"FPS: {result['fps']:.1f}", (10, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(frame, f"Frame: {result['frame']}", (10, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    
    # Texture bar
    bar_width = int(min(result['texture'] * 5, 100))
    threshold_x = int(result['texture'] * 5)
    cv2.rectangle(frame, (w-120, 30), (w-20, 40), (50, 50, 50), -1)
    cv2.rectangle(frame, (w-120, 30), (w-120 + bar_width, 40), 
                 (0, 255, 0) if result['texture'] > 4 else (0, 165, 255), -1)
    
    # Indikator metode
    if "Phase" in result['method']:
        cv2.circle(frame, (w-25, 60), 10, (0, 165, 255), -1)
        cv2.putText(frame, "PLAIN", (w-70, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
    else:
        cv2.circle(frame, (w-25, 60), 10, (0, 255, 0), -1)
        cv2.putText(frame, "TEXT", (w-65, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

def main():
    print("=" * 60)
    print("OPTIMIZED VISUAL ODOMETRY")
    print("=" * 60)
    print("\nTips untuk hasil terbaik:")
    print("  1. Atur kamera pada ketinggian 20-30 cm dari permukaan")
    print("  2. Pastikan permukaan memiliki kontras (bukan polos)")
    print("  3. Gunakan kertas koran/karpet berpola sebagai alas")
    print("  4. Gerakkan kamera perlahan")
    print("=" * 60)
    
    vo = OptimizedVisualOdometry(camera_index=1)
    
    if not vo.connect():
        print("\nError: Gagal konek ke kamera!")
        return
    
    print("\nKontrol:")
    print("  [q] Keluar")
    print("  [r] Reset posisi")
    print("  [s] Screenshot")
    print("  [t] Tampilkan statistik")
    print("=" * 60)
    
    history_x = []
    history_y = []
    
    try:
        while True:
            frame = vo.get_frame()
            if frame is None:
                continue
            
            result = vo.process(frame)
            
            if result:
                # Update history
                history_x.append(result['pos_x'])
                history_y.append(result['pos_y'])
                if len(history_x) > 500:
                    history_x.pop(0)
                    history_y.pop(0)
                
                # Gambar trajectory
                for i in range(1, len(history_x)):
                    x1 = int(history_x[i-1] % 600 + 50)
                    y1 = int(history_y[i-1] % 400 + 100)
                    x2 = int(history_x[i] % 600 + 50)
                    y2 = int(history_y[i] % 400 + 100)
                    cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                
                # Titik posisi saat ini
                current_x = int(result['pos_x'] % 600 + 50)
                current_y = int(result['pos_y'] % 400 + 100)
                cv2.circle(frame, (current_x, current_y), 8, (0, 0, 255), -1)
                
                # Info
                stats = vo.get_stats()
                draw_info(frame, result, stats)
                
                # Tambahan info untuk debugging
                if result['texture'] < 4:
                    cv2.putText(frame, "LOW TEXTURE! Use patterned surface", (10, 250),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
                cv2.imshow('Optimized Visual Odometry', frame)
                
                # Output terminal
                if result['frame'] % 30 == 0:
                    print(f"[{result['frame']}] "
                          f"Pos: ({result['pos_x_mm']:.1f}, {result['pos_y_mm']:.1f}) mm | "
                          f"Texture: {result['texture']:.1f} | "
                          f"{result['method']} | {result['fps']:.1f} FPS")
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                vo.reset()
                history_x = []
                history_y = []
                print("Posisi direset")
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"odometry_{timestamp}.png", frame)
                print(f"Screenshot: odometry_{timestamp}.png")
            elif key == ord('t'):
                stats = vo.get_stats()
                print(f"\nStatistik:")
                print(f"  Low texture: {stats['low_texture_percent']:.1f}%")
                print(f"  No movement: {stats['no_movement_percent']:.1f}%")
    
    except KeyboardInterrupt:
        print("\nProgram dihentikan")
    finally:
        vo.release()
        cv2.destroyAllWindows()
        
        stats = vo.get_stats()
        print("\n" + "=" * 60)
        print("HASIL AKHIR")
        print("=" * 60)
        print(f"Posisi X: {vo.pos_x * vo.pixel_to_mm:.2f} mm")
        print(f"Posisi Y: {vo.pos_y * vo.pixel_to_mm:.2f} mm")
        print(f"Total frame: {vo.total_frames}")
        print(f"Low texture frames: {stats['low_texture_percent']:.1f}%")
        print("=" * 60)

if __name__ == "__main__":
    main()