import cv2
import numpy as np
import time
from collections import deque

class VisualOdometry:
    def __init__(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.prev_gray = None
        self.flow_history = deque(maxlen=5)  # Smoothing dengan moving average
        
    def process_frame(self, frame):
        # Konversi ke grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            return None
        
        # Hitung dense optical flow dengan metode Farneback
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None,
            pyr_scale=0.5,      # Pyramid scale
            levels=3,           # Number of pyramid levels
            winsize=15,         # Window size
            iterations=3,       # Iterations per level
            poly_n=5,           # Polynomial size
            poly_sigma=1.2,     # Polynomial sigma
            flags=0
        )
        
        # Hitung pergerakan rata-rata dari semua piksel
        # Ambil hanya area tengah frame (menghindari noise di tepi)
        h, w = flow.shape[:2]
        margin_h = int(h * 0.2)
        margin_w = int(w * 0.2)
        
        flow_center = flow[margin_h:h-margin_h, margin_w:w-margin_w]
        
        # Rata-rata pergerakan
        dx = np.mean(flow_center[:, :, 0])
        dy = np.mean(flow_center[:, :, 1])
        
        # Smoothing dengan moving average
        self.flow_history.append((dx, dy))
        dx_smooth = np.mean([f[0] for f in self.flow_history])
        dy_smooth = np.mean([f[1] for f in self.flow_history])
        
        # Akumulasi posisi
        self.pos_x += dx_smooth
        self.pos_y += dy_smooth
        
        # Update frame sebelumnya
        self.prev_gray = gray
        
        return {
            'dx': dx_smooth,
            'dy': dy_smooth,
            'pos_x': self.pos_x,
            'pos_y': self.pos_y,
            'flow': flow
        }
    
    def reset_position(self):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.flow_history.clear()

def draw_flow(frame, flow, step=16):
    """Visualisasi optical flow dengan panah"""
    h, w = flow.shape[:2]
    y, x = np.mgrid[step//2:h:step, step//2:w:step].reshape(2, -1).astype(int)
    fx, fy = flow[y, x].T
    
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    
    for (x1, y1), (x2, y2) in lines:
        cv2.arrowedLine(frame, (x1, y1), (x2, y2), (0, 255, 0), 1, tipLength=0.3)
    
    return frame

def main():
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Tidak bisa membuka kamera")
        return
    
    # Set resolusi lebih kecil untuk performa lebih baik
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    vo = VisualOdometry()
    
    print("Mulai Visual Odometry (Farneback)...")
    print("Tekan 'r' untuk reset posisi")
    print("Tekan 'q' untuk keluar")
    print("Posisi dalam satuan pixel (belum dikalibrasi ke mm)")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Proses frame
        result = vo.process_frame(frame)
        
        if result:
            # Tampilkan informasi di frame
            cv2.putText(frame, f"X: {result['pos_x']:.1f} px", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Y: {result['pos_y']:.1f} px", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"dX: {result['dx']:.2f}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, f"dY: {result['dy']:.2f}", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Visualisasi flow
            frame = draw_flow(frame, result['flow'], step=32)
        
        cv2.imshow('Visual Odometry - Farneback', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            vo.reset_position()
            print("Posisi direset")
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"\nPosisi akhir - X: {vo.pos_x:.2f} px, Y: {vo.pos_y:.2f} px")

if __name__ == "__main__":
    main()