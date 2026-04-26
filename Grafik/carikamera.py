import cv2
import time

def identify_cameras():
    """Identifikasi kamera yang terdeteksi dengan preview"""
    
    print("=" * 60)
    print("IDENTIFIKASI KAMERA")
    print("=" * 60)
    
    cameras_info = []
    
    for idx in [0, 1, 2]:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                h, w = frame.shape[:2]
                print(f"\nCamera Index {idx}:")
                print(f"  Resolution: {w}x{h}")
                print(f"  Showing preview for 3 seconds...")
                
                # Tampilkan preview dengan label
                start_time = time.time()
                while time.time() - start_time < 3:
                    ret, frame = cap.read()
                    if ret:
                        # Tambahkan label
                        cv2.putText(frame, f"Camera Index: {idx}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(frame, "This window will close in 3 seconds", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.imshow(f'Camera {idx}', frame)
                        cv2.waitKey(1)
                
                cv2.destroyWindow(f'Camera {idx}')
                cameras_info.append(idx)
            cap.release()
    
    print("\n" + "=" * 60)
    print("Sekarang gerakkan HP Anda!")
    print("Script akan menampilkan preview masing-masing kamera lagi")
    print("Lihat kamera mana yang bergerak sesuai dengan gerakan HP Anda")
    print("=" * 60)
    input("Tekan ENTER untuk melanjutkan...")
    
    # Preview lebih lama untuk identifikasi
    for idx in cameras_info:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            print(f"\nMenampilkan Camera {idx} - Tekan 'q' untuk lanjut ke kamera berikutnya")
            
            while True:
                ret, frame = cap.read()
                if ret:
                    cv2.putText(frame, f"Camera Index: {idx}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, "Gerakkan HP Anda", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame, "Tekan 'q' jika ini adalah DroidCam", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.imshow(f'Camera {idx} - DroidCam?', frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print(f"Camera {idx} dipilih sebagai DroidCam!")
                        cap.release()
                        cv2.destroyAllWindows()
                        return idx
            cap.release()
    
    cv2.destroyAllWindows()
    return None

if __name__ == "__main__":
    droidcam_idx = identify_cameras()
    
    if droidcam_idx is not None:
        print("\n" + "=" * 60)
        print(f"KESIMPULAN")
        print(f"DroidCam terdeteksi di Camera Index: {droidcam_idx}")
        print("=" * 60)
        
        # Simpan ke file untuk digunakan nanti
        with open("camera_index.txt", "w") as f:
            f.write(str(droidcam_idx))
        print("Index kamera disimpan ke camera_index.txt")
    else:
        print("\nTidak ada kamera yang dipilih")