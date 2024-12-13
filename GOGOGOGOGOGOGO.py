# KELOMPOK 3 TENGAH
import collections
try:
    from collections import abc
    print("READY")
    collections.MutableMapping = abc.MutableMapping
except:
    pass

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time
from ultralytics import YOLO
import cv2
import numpy as np
import math

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


def add_last_waypoint_to_mission(wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude):
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
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

    return (cmds.count)


def gerak(vx, vy):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def process_video_with_navigation(video_source, model_path, conf_threshold=0.60, iou_threshold=0.80,
                                   angle_threshold=10, lost_detection_timeout=3, window_name="Navigation Output"):
    model = YOLO(model_path)
    model.conf = conf_threshold
    model.iou = iou_threshold

    prev_time = time.time()
    last_detection_time = time.time()
    cap = cv2.VideoCapture(video_source)  # Menggunakan webcam atau file video

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from video")
            break

        frame_height, frame_width = frame.shape[:2]
        curr_time = time.time()
        fps = int(1 / (curr_time - prev_time))
        prev_time = curr_time

        cx = frame_width // 2
        cy = frame_height // 2
        frame_center = (cx, cy)

        cv2.line(frame, (cx, 0), (cx, frame_height), (0, 255, 0), 2)
        cv2.circle(frame, frame_center, 5, (0, 0, 255), -1)

        results = model(frame)
        red_ball = []
        green_ball = []

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                label = model.names[class_id]
                confidence = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                if label == 'merah':
                    red_ball.append({"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1})
                elif label == 'hijau':
                    green_ball.append({"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1})

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

        for cx, cy in centroid_merah:
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        for cx, cy in centroid_hijau:
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

        annotated_frame = frame.copy()
        frame_bottom_center = (frame_width // 2, frame_height)

        red_ball_closest = sorted(centroid_merah, key=lambda x: x[1], reverse=True)[:1]
        green_ball_closest = sorted(centroid_hijau, key=lambda x: x[1], reverse=True)[:1]

        if red_ball_closest:
            cv2.line(annotated_frame, frame_bottom_center, red_ball_closest[0], (0, 0, 255), 2)
        if green_ball_closest:
            cv2.line(annotated_frame, frame_bottom_center, green_ball_closest[0], (0, 255, 0), 2)
        if red_ball_closest and green_ball_closest:
            cv2.line(annotated_frame, red_ball_closest[0], green_ball_closest[0], (255, 255, 255), 2)

        navigation_command = "BERHENTI"
        status_text = "Tidak ada bola terdeteksi"
        groundspeed = 0.5

        if not red_ball and not green_ball:
            if curr_time - last_detection_time > lost_detection_timeout:
                navigation_command = "BERHENTI"
                status_text = "Tidak ada bola terdeteksi - BERHENTI"
                gerak(0, 0)

        elif red_ball and not green_ball:
            last_detection_time = curr_time
            navigation_command = "BELOK KANAN"
            status_text = "Mencari bola hijau - BELOK KANAN"
            gerak(groundspeed, 1)

        elif green_ball and not red_ball:
            last_detection_time = curr_time
            navigation_command = "BELOK KIRI"
            status_text = "Mencari bola merah - BELOK KIRI"
            gerak(groundspeed, -1)

        elif red_ball and green_ball:
            last_detection_time = curr_time
            pair_center = ((red_ball_closest[0][0] + green_ball_closest[0][0]) // 2,
                           (red_ball_closest[0][1] + green_ball_closest[0][1]) // 2)

            dx = pair_center[0] - cx
            angle = np.arctan2(dx, frame_height - pair_center[1]) * 180 / np.pi

            if abs(angle) < angle_threshold:
                navigation_command = "LURUS"
                status_text = f"Menuju titik tengah - LURUS (Sudut: {angle:.1f})"
                gerak(groundspeed, 0)
            elif angle > 0:
                navigation_command = "BELOK KANAN"
                status_text = f"Menuju titik tengah - BELOK KANAN (Sudut: {angle:.1f})"
                gerak(groundspeed, math.sin(angle))
            else:
                navigation_command = "BELOK KIRI"
                status_text = f"Menuju titik tengah - BELOK KIRI (Sudut: {angle:.1f})"
                gerak(groundspeed, math.sin(angle))

            cv2.circle(annotated_frame, pair_center, 5, (255, 255, 0), -1)
            cv2.putText(annotated_frame, f"Sudut: {angle:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.putText(annotated_frame, f"Status: {status_text}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(annotated_frame, f"Perintah: {navigation_command}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow(window_name, annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# Gantilah "model_path_here" dengan path model YOLO yang kamu miliki.
model_path = r"D:\Magang UV NAVAL\Foto Tes\FINAL.pt"

# Gantilah "0" jika kamu ingin menggunakan webcam lain (seperti 1, 2, dst)
video_source = r"D:\vidioPOV1\vidioPOV1.mp4" # Menggunakan webcam pertama

process_video_with_navigation(video_source, model_path)
