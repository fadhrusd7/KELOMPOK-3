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
vehicle = connect('tcp:127.0.0.1:5762')

print("WAITING..")

def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
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

def add_last_waypoint_to_mission(wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude):
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    missionlist = []
    for cmd in cmds:
        missionlist.append(cmd)

    wpLastObject = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                           0, 0, 0, 0, 0, 0,
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    cmds.clear()

    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

    return cmds.count

def gerak(vx, vy):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        int((time.time() % 1) * 1000),
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
    model = YOLO("FINAL.pt")
    prev_time = time.time()
    cap = cv2.VideoCapture("vidioPOV1.mp4")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from video")
            break
        model.conf = 0.65
        
        frame_height, frame_width = frame.shape[:2]
        
        curr_time = time.time()
        fps = int(1 / (curr_time - prev_time))
        prev_time = curr_time
        
        cx = frame_width // 2
        cy = frame_height // 2
        frame_center = (cx, cy)
        
        cv2.line(frame, (cx, 0), (cx, frame_height), (0, 255, 0), 2)
        cv2.circle(frame, frame_center, 5, (0, 0, 255), -1)
        
        results = model(frame, conf=0.4)
        red_ball = []
        green_ball = []
        
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                label = model.names[class_id]
                confidence = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                if label == 'merah':
                    red_ball.append({"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1})
                elif label == 'hijau':
                    green_ball.append({"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1})
                    
        centroid_merah = [(ball["x"] + ball["w"] // 2, ball["y"] + ball["h"] // 2) for ball in red_ball]
        centroid_hijau = [(ball["x"] + ball["w"] // 2, ball["y"] + ball["h"] // 2) for ball in green_ball]
        
        for cx, cy in centroid_merah:
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        for cx, cy in centroid_hijau:
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        
        annotated_frame = frame.copy()
        frame_bottom_center = (frame_width // 2, frame_height)
        
        red_ball_closest = sorted(centroid_merah, key=lambda x: x[1], reverse=True) if red_ball else [None]
        green_ball_closest = sorted(centroid_hijau, key=lambda x: x[1], reverse=True) if green_ball else [None]
        
        if red_ball_closest[0] is not None:
            cv2.line(annotated_frame, frame_bottom_center, red_ball_closest[0], (0, 0, 255), 2)
        if green_ball_closest[0] is not None:
            cv2.line(annotated_frame, frame_bottom_center, green_ball_closest[0], (0, 255, 0), 2)
        if red_ball_closest[0] is not None and green_ball_closest[0] is not None:
            cv2.line(annotated_frame, red_ball_closest[0], green_ball_closest[0], (255, 255, 255), 2)

        centroid_bola = []
        if red_ball_closest[0] is not None and green_ball_closest[0] is not None:
            pair_center = ((red_ball_closest[0][0] + green_ball_closest[0][0]) // 2, 
                           (red_ball_closest[0][1] + green_ball_closest[0][1]) // 2)
            centroid_bola.append(pair_center)
            cv2.line(annotated_frame, frame_bottom_center, centroid_bola[0], (255, 255, 0), 2)
            
        turn_speed = 0.3
        belok_kiri = -0.3
        belok_kanan = 0.3
        berhenti = 0.0 
     
        if centroid_bola:
            print("ada bola pasangan terdeteksi")
            ball_center = centroid_bola[0]
            dx = ball_center[0] - frame_bottom_center[0]
            dy = frame_bottom_center[1] - ball_center[1]
            vy = math.atan2(dx, dy)
            vx = turn_speed 
            cv2.putText(annotated_frame, f"vx: {vx:.2f}, vy: {vy:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) 
            gerak(turn_speed, vy)
        else:
            print("TIDAK MELIHAT PASANGAN BOLA")
            if red_ball_closest[0] is not None and green_ball_closest[0] is None:
                vx = turn_speed
                vy = belok_kanan
                gerak(turn_speed, belok_kanan)
                cv2.putText(annotated_frame, f"vx: {vx:.2f}, vy: {vy:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                print("BELOK KANAN")
            elif green_ball_closest[0] is not None and red_ball_closest[0] is None:
                vx = turn_speed
                vy = belok_kiri
                gerak(turn_speed, belok_kiri)
                cv2.putText(annotated_frame, f"vx: {vx:.2f}, vy: {vy:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)  
                print("BELOK KIRI")
            else:
                print("Tidak ada bola")
                if red_ball and green_ball and not centroid_bola:
                    print("Tidak ada bola, maju perlahan.")
                    gerak(berhenti, turn_speed)
