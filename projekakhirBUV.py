import cv2
import numpy as np

def detect_balls(frame, min_ball_area=100):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range untuk bola merah
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    
    # Define range untuk bola hijau
    lower_green = np.array([80, 70, 50])
    upper_green = np.array([95, 255, 255])
    
    # Membuat mask untuk masing-masing warna
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    # Morphological operations untuk noise reduction
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    
    # Menemukan contours
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Menggambar hasil deteksi
    result = frame.copy()
    
    # Deteksi bola merah
    for cnt in contours_red:
        area = cv2.contourArea(cnt)
        if area > min_ball_area:  # Gunakan parameter ukuran minimal
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(result, center, radius, (0, 0, 255), 2)
            cv2.putText(result, 'Red Ball', (int(x - radius), int(y - radius)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Deteksi bola hijau
    for cnt in contours_green:
        area = cv2.contourArea(cnt)
        if area > min_ball_area:  # Gunakan parameter ukuran minimal
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(result, center, radius, (0, 255, 0), 2)
            cv2.putText(result, 'Green Ball', (int(x - radius), int(y - radius)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Membagi frame sesuai grid di gambar
    height, width, _ = frame.shape
    top_height = height // 3  # Bagian atas
    middle_height = height // 3  # Bagian tengah
    bottom_height = height - (top_height + middle_height)  # Sisa untuk bagian bawah
    col_width = width // 3  # Lebar setiap kolom

    #   Garis horizontal
    cv2.line(result, (0, top_height), (width, top_height), (255, 255, 255), 2)  # Garis bawah bagian atas
    cv2.line(result, (0, top_height + middle_height), (width, top_height + middle_height), (255, 255, 255), 2)  # Garis bawah bagian tengah

    # Garis vertikal
    cv2.line(result, (col_width, top_height), (col_width, height), (255, 255, 255), 2)  # Garis vertikal pertama
    cv2.line(result, (2 * col_width, top_height), (2 * col_width, height), (255, 255, 255), 2)  # Garis vertikal kedua

    # Menambahkan label manual
    #   Label bagian atas (R1)
    cv2.putText(result, "R1", (width // 2, top_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Label bagian tengah (R2C1, R2C2, R2C3)
    cv2.putText(result, "R2C1", (col_width // 2, top_height + middle_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(result, "R2C2", (col_width + col_width // 2, top_height + middle_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(result, "R2C3", (2 * col_width + col_width // 2, top_height + middle_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Label bagian bawah (R3C1, R3C2, R3C3)
    cv2.putText(result, "R3C1", (col_width // 2, top_height + middle_height + bottom_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(result, "R3C2", (col_width + col_width // 2, top_height + middle_height + bottom_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(result, "R3C3", (2 * col_width + col_width // 2, top_height + middle_height + bottom_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    
    return result


video_path = "videokapal.mp4"
cap = cv2.VideoCapture(video_path)

# Minimal ukuran bola dalam piksel persegi
MIN_BALL_AREA = 200  # Ubah nilai sesuai kebutuhan

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    result = detect_balls(frame, min_ball_area=MIN_BALL_AREA)
    cv2.imshow('Result', result)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
