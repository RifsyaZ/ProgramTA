import cv2
import numpy as np
import time

def main():
    # Buka kamera laptop (0 = kamera default)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Tidak bisa membuka kamera")
        return
    
    # Ambil frame pertama
    ret, frame = cap.read()
    if not ret:
        print("Error: Tidak bisa membaca frame")
        return
    
    # Konversi ke grayscale
    prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Parameter untuk Lucas-Kanade optical flow
    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    # Pilih titik-titik yang akan dilacak (menggunakan grid)
    feature_params = dict(maxCorners=100,
                          qualityLevel=0.3,
                          minDistance=7,
                          blockSize=7)
    
    # Inisialisasi titik awal
    p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
    
    # Warna untuk visualisasi
    color = (0, 255, 0)
    
    # Variabel untuk posisi kumulatif
    pos_x = 0
    pos_y = 0
    
    print("Mulai visual odometry... Tekan 'q' untuk berhenti")
    print("Posisi X: 0.00, Posisi Y: 0.00")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Konversi ke grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Hitung optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **lk_params)
        
        # Pilih titik yang berhasil dilacak
        good_new = p1[st == 1]
        good_old = p0[st == 1]
        
        # Hitung pergerakan rata-rata
        if len(good_new) > 0:
            # Hitung perubahan posisi (dx, dy) dari semua titik
            dx = np.mean(good_new[:, 0] - good_old[:, 0])
            dy = np.mean(good_new[:, 1] - good_old[:, 1])
            
            # Akumulasi posisi (dalam pixel)
            pos_x += dx
            pos_y += dy
            
            # Tampilkan posisi di terminal
            print(f"Posisi X: {pos_x:.2f}, Posisi Y: {pos_y:.2f}")
            
            # Visualisasi: gambar garis pergerakan
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel().astype(int)
                c, d = old.ravel().astype(int)
                cv2.line(frame, (a, b), (c, d), color, 2)
                cv2.circle(frame, (a, b), 3, color, -1)
        
        # Tampilkan posisi di frame
        cv2.putText(frame, f"X: {pos_x:.1f} px", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Y: {pos_y:.1f} px", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Tampilkan frame
        cv2.imshow('Visual Odometry - Lucas-Kanade', frame)
        
        # Update frame sebelumnya
        prev_gray = gray.copy()
        p0 = good_new.reshape(-1, 1, 2)
        
        # Reset titik jika terlalu sedikit
        if len(p0) < 10:
            p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
        
        # Tekan 'q' untuk keluar
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"\nPosisi akhir - X: {pos_x:.2f} px, Y: {pos_y:.2f} px")

if __name__ == "__main__":
    main()