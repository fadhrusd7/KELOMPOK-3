import collections
from ultralytics import YOLO
import cv2
import math
try:
    from collections import abc
    print("READY")
    collections.MutableMapping = abc.MutableMapping
except ImportError:
    pass

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time

print('Connecting...')
# vehicle = connect('tcp:192.168.229.171:5762')
vehicle = connect('tcp:127.0.0.1:5762')
# vehicle = connect('/dev/ttyACM0')
# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

print("WAITING..")

def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        # time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break


def add_last_waypoint_to_mission(  wp_Last_Latitude,
        wp_Last_Longitude,
        wp_Last_Altitude):  # --- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Save the vehicle commands to a list
    missionlist = []
    for cmd in cmds:
        missionlist.append(cmd)

    # Modify the mission as needed. For example, here we change the
    wpLastObject = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                           0, 0, 0, 0, 0, 0,
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    # Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

    return (cmds.count)


def gerak(vx, vy):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        int((time.time() % 1) * 1000),  # Convert time to milliseconds and ensure it is within valid range
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
def process_frame_with_navigation():
    """
    Fungsi untuk memproses frame dari video capture dengan menampilkan hasil deteksi objek menggunakan YOLO
    """
    
    model = YOLO("FINAL.pt") # Load model YOLO
    # Mengatur treshold untuk deteksi objek
   
    
    prev_time = time.time()
    # Load kamera
    cap = cv2.VideoCapture("vidioPOV1.mp4")
    
    while True:
        # Membaca frame dari video capture
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from video")
            break
        model.conf = 0.65
       
        
        frame_height, frame_width = frame.shape[:2]
        
        # Hitung FPS berdasarkan waktu antar frame
        curr_time = time.time()
        fps = int(1 / (curr_time - prev_time))
        prev_time = curr_time
       
        
        # Menentukan pusat frame
        cx = frame_width // 2
        cy = frame_height // 2
        frame_center = (cx, cy)
        
        # gambar garis tengah frame 
        cv2.line(frame, (cx, 0), (cx, frame_height), (0, 255, 0), 2)
        
        # Menggambar titik tengah frame
        cv2.circle(frame, frame_center, 5, (0, 0, 255), -1)
        
        # Deteksi objek menggunakan YOLO
        results = model(frame, conf = 0.4)
        red_ball = []
        green_ball = []
        
        # menampilkan hasil deteksi objek serta confidence score
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                label = model.names[class_id]
                confidence = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        
        
        # Mendeteksi dan menyimpan bola merah dan bola hijau yang ditandai dengan class "merah" dan "hijau"
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                label = model.names[class_id]
                if label == 'merah':
                    red_ball.append({"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1})
                elif label == 'hijau':
                    green_ball.append({"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1})
                    
        # hitung centroid red_ball dan green_ball berdasar kan nilai x1 x2 dan y1 y2
        centroid_merah = []
        centroid_hijau = []
        
        for ball in red_ball:
            ball["x"] += ball["w"] // 2
            ball["y"] += ball["h"] // 2
            centroid_merah.append((ball["x"], ball["y"]))

        for ball in green_ball:
            ball["x"] += ball["w"] // 2
            ball["y"] += ball["h"] // 2
            centroid_hijau.append((ball["x"], ball["y"]))
            
       # mengambar centroid bola merah dan hijau
        for cx, cy in centroid_merah:
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        for cx, cy in centroid_hijau:
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        
    
        annotated_frame = frame.copy()
        # menentukan titik tengah bawah frame
        frame_bottom_center = (frame_width // 2, frame_height)
    
        #!SECTION DETEKSI BOLA MERAH DAN HIJAU TERDEKAT             
        # membuat list untuk menyimpan bola merah dan hijau terdekat
        red_ball_closest = [None]
        green_ball_closest = [None]
        
        # mencari bola merah dan hijau terdekat berdasarkan nilai y terbesar
        if red_ball:
            red_ball_closest = sorted(centroid_merah, key=lambda x: x[1], reverse=True)
        if green_ball:
            green_ball_closest = sorted(centroid_hijau, key=lambda x: x[1], reverse=True)
            
        
        # Menggambar garis dari titik frame_bottom_center ke setiap ke bola merah dan hijau terdekat
        if red_ball_closest[0] is not None:
            cv2.line(annotated_frame, frame_bottom_center, red_ball_closest[0], (0, 0, 255), 2)
        if green_ball_closest[0] is not None:
            cv2.line(annotated_frame, frame_bottom_center, green_ball_closest[0], (0, 255, 0), 2)
        # Menggambar garis antara red_ball_closest dan green_ball_closest
        if red_ball_closest[0] is not None and green_ball_closest[0] is not None:
            cv2.line(annotated_frame, red_ball_closest[0], green_ball_closest[0], (255, 255, 255), 2)


        # Fungsi untuk menghitung jarak Euclidean antara dua titik
        # Mendefinisikan threshold jarak yang dianggap sebagai pasangan bola yang valid
        # valid_distance_threshold = 1000 # Misalnya, bola dianggap pasangan jika jaraknya kurang dari 100 piksel
    
   
        
        # Menentukan  koordinat titik tengah pasangan red_ball_closest dan green_ball_closest
        centroid_bola = []
        if red_ball_closest[0] is not None and green_ball_closest[0] is not None:
            pair_center = ((red_ball_closest[0][0] + green_ball_closest[0][0]) // 2, 
                       (red_ball_closest[0][1] + green_ball_closest[0][1]) // 2)
            centroid_bola.append(pair_center)
            cv2.line(annotated_frame, frame_bottom_center, centroid_bola[0], (255, 255, 0), 2)
            
        #!SECTION GERAK KAPAL BERDASARKAN POSISI BOLA
        # max_speed = 0.6  # Kecepatan maksimal
        turn_speed = 0.3 # Kecepatan untuk belokan kecil
        belok_kiri = -0.3
        belok_kanan = 0.3
        berhenti = 0.0 
     

        if len(centroid_bola) > 0:
            print("ada bola pasangan terdeteksi")
            if centroid_bola:
                 # Ambil centroid bola pasangan
                ball_center = centroid_bola[0]

                # Hitung dx dan dy
                dx = ball_center[0] - frame_bottom_center[0]
                dy = frame_bottom_center[1] - ball_center[1]

                # Hitung sudut dalam radian
                vy = math.atan2(dx, dy)
            # Kecepatan maju tetap
            vx = turn_speed 
            cv2.putText(annotated_frame, f"vx: {vx:.2f}, vy: {vy:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) 
            gerak(turn_speed, vy)
        
        if len(centroid_bola) == 0 :
            print("TIDAK MELIHAT PASANGAN BOLA")
            if red_ball_closest[0] is not None and green_ball_closest[0] is None:  # Hanya bola merah
                    vx = turn_speed
                    vy = belok_kanan
                    gerak(turn_speed, belok_kanan)
                    cv2.putText(annotated_frame, f"vx: {vx:.2f}, vy: {vy:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    print("BELOK KANAN")
            elif green_ball_closest[0] is not None and red_ball_closest[0] is None:  # Hanya bola hijau
                    vx = turn_speed
                    vx = belok_kiri
                    gerak(turn_speed, belok_kiri)
                    cv2.putText(annotated_frame, f"vx: {vx:.2f}, vy: {vy:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  
                    print("BELOK KIRI")

            #APABILA TIDAK TERLIHAT BOLA
            else:
                print("Tidak ada bola")
                if  red_ball and green_ball and centroid_bola is None:  # Start tanpa bola
                    print("Tidak ada bola, maju perlahan.")
                    gerak(berhenti,turn_speed)
                    cv2.putText(annotated_frame, f"vx: {berhenti:.2f}, vy: {turn_speed:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
             # Finish setelah zigzag
                elif red_ball and green_ball is None and centroid_bola is not None:  # Menggunakan koordinat terakhir
                    print("Tidak ada bola, mengikuti koordinat terakhir.")
                    vy = belok_kanan
                    gerak(turn_speed, belok_kiri)
                    cv2.putText(annotated_frame, f"vx: {turn_speed:.2f}, vy: {vy:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    # Pemberhentian setelah 5 detik
                    time.sleep(5)
                    gerak(turn_speed, belok_kanan)
                    cv2.putText(annotated_frame, f"vx: {berhenti:.2f}, vy: {berhenti:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    time.sleep(5)
                    gerak(berhenti, berhenti)
                    cv2.putText(annotated_frame, f"vx: {berhenti:.2f}, vy: {berhenti:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                else:
                    gerak(berhenti,berhenti)
                    cv2.putText(annotated_frame, f"vx: {berhenti:.2f}, vy: {berhenti:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    print("STOP")
                
       
       
        # Menampilkan frame hasil deteksi
        #tamplkan FPS pada frame
        cv2.putText(annotated_frame, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # Tampilkan output gerak robot pada annotated frame
        cv2.putText(annotated_frame, "gerak", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        resized_frame = cv2.resize(annotated_frame, (1080, 720))
        cv2.imshow("Annotated Frame", resized_frame)
        
        if cv2.waitKey(15) & 0xFF == ord("q"):
            break
        
# Menjalankan fungsi
process_frame_with_navigation()


if __name__ == "__main__":
    add_last_waypoint_to_mission(vehicle.location.global_relative_frame.lat,
                                 vehicle.location.global_relative_frame.lon,
                                 vehicle.location.global_relative_frame.alt)
    print("Home waypoint added to the mission")
    arm_and_takeoff(0)
    print("READY")
    ground_speed = 2
    while True :
        gerak(ground_speed, 0)
        print("GOO")
    