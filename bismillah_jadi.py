import cv2
import numpy as np
from code_mavlink import gerak

def get_frame_region(x, y, frame_height, frame_width):
    """Menentukan region berdasarkan koordinat"""
    top_height = frame_height // 3
    middle_height = frame_height // 3
    col_width = frame_width // 3
    
    if y > top_height and y <= (top_height + middle_height) and x <= col_width:
        return "R2C1"
    elif y > top_height and y <= (top_height + middle_height) and x > col_width and x <= 2*col_width:
        return "R2C2"
    elif y > top_height and y <= (top_height + middle_height) and x > 2*col_width:
        return "R2C3"
    return "OTHER"

def navigate_based_on_balls(red_positions, green_positions):
    """Fungsi untuk menentukan gerak berdasarkan posisi bola"""
    ground_speed = 2  # Kecepatan dasar
    
    if "R2C1" in red_positions and "R2C3" in green_positions:
        gerak(ground_speed, 0)
        return "FORWARD"
    elif "R2C2" in green_positions:
        gerak(0, -ground_speed)
        return "LEFT"
    elif "R2C2" in red_positions:
        gerak(0, ground_speed)
        return "RIGHT"
    else:
        gerak(ground_speed, 0)
        return "DEFAULT_FORWARD"

def detect_balls_with_navigation(frame, min_ball_area=100):
    height, width, _ = frame.shape
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define color ranges
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_green = np.array([80, 70, 50])
    upper_green = np.array([95, 255, 255])
    
    # Create masks
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    # Process masks
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    
    # Find contours
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    result = frame.copy()
    red_positions = []
    green_positions = []
    
    # Process red balls
    for cnt in contours_red:
        area = cv2.contourArea(cnt)
        if area > min_ball_area:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            region = get_frame_region(x, y, height, width)
            red_positions.append(region)
            
            # Draw detection
            cv2.circle(result, center, radius, (0, 0, 255), 2)
            cv2.putText(result, 'Red Ball', (int(x - radius), int(y - radius)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Process green balls
    for cnt in contours_green:
        area = cv2.contourArea(cnt)
        if area > min_ball_area:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            region = get_frame_region(x, y, height, width)
            green_positions.append(region)
            
            # Draw detection
            cv2.circle(result, center, radius, (0, 255, 0), 2)
            cv2.putText(result, 'Green Ball', (int(x - radius), int(y - radius)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Navigasi berdasarkan posisi bola
    navigation_command = navigate_based_on_balls(red_positions, green_positions)
    
    # Tampilkan perintah navigasi pada frame
    cv2.putText(result, f"Navigation: {navigation_command}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Draw grid
    draw_grid(result, height, width)
    
    return result

def draw_grid(frame, height, width):
    top_height = height // 3
    middle_height = height // 3
    col_width = width // 3
    
    # Draw horizontal lines
    cv2.line(frame, (0, top_height), (width, top_height), (255, 255, 255), 2)
    cv2.line(frame, (0, top_height + middle_height), (width, top_height + middle_height), (255, 255, 255), 2)
    
    # Draw vertical lines
    cv2.line(frame, (col_width, top_height), (col_width, height), (255, 255, 255), 2)
    cv2.line(frame, (2 * col_width, top_height), (2 * col_width, height), (255, 255, 255), 2)
    
    # Add labels
    cv2.putText(frame, "R1", (width // 2, top_height // 2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, "R2C1", (col_width // 2, top_height + middle_height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, "R2C2", (col_width + col_width // 2, top_height + middle_height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, "R2C3", (2 * col_width + col_width // 2, top_height + middle_height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, "R3C1", (col_width // 2, top_height + middle_height + (height - top_height - middle_height) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, "R3C2", (col_width + col_width // 2, top_height + middle_height + (height - top_height - middle_height) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, "R3C3", (2 * col_width + col_width // 2, top_height + middle_height + (height - top_height - middle_height) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

# Main loop
if __name__ == "__main__":
    video_path = "videokapal.mp4"
    cap = cv2.VideoCapture(video_path)
    MIN_BALL_AREA = 200

    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        result = detect_balls_with_navigation(frame, min_ball_area=MIN_BALL_AREA)
        cv2.imshow('Result', result)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()