#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import threading
import serial as pyserial
from collections import deque
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R_scipy
import openpyxl
from datetime import datetime

import os
import sys
devnull = open(os.devnull, 'w')
os.dup2(devnull.fileno(), 2)

#####################################################################################################
########### CONFIGURACION - CAMBIAR SEGUN NECESIDAD

TRACK_TAG_ID = 8                    #####   CHANGE AS REQUIRED
WORLD_TAG_ID = 9                    #####   CHANGE AS REQUIRED
FRAME_RATE = 60
world_tag_size = 0.15               #####   CHANGE AS REQUIRED
target_tag_size = 0.15              #####   CHANGE AS REQUIRED

ARDUINO_PORT = "COM8"               #####   CHANGE AS REQUIRED
ARDUINO_BAUD = 115200

timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
EXCEL_FILENAME = f"grinding_data_{timestamp_str}.xlsx"
PRINT_INTERVAL = 1
VIDEO_CODEC = cv2.VideoWriter_fourcc(*'XVID')

#####################################################################################################
########### Configuracion del detector de april tags

detector_kwargs = dict(
    families="tag16h5",
    nthreads=2,
    quad_decimate=1.0,
    quad_sigma=0.8,
    refine_edges=True,
    decode_sharpening=0.25,
)
detector = Detector(**detector_kwargs)

#####################################################################################################
########### Almacenamiento de datos en memoria

camera_rows = []
camera_rows_lock = threading.Lock()
arduino_rows = []
arduino_rows_lock = threading.Lock()
last_print_time = {}
current_pair_time = 0.0

#####################################################################################################
########### Funciones de calculo de pose y velocidad

def is_valid_rotation_matrix(matrix):
    det = np.linalg.det(matrix)
    return det > 0.9

def rotmat_to_quat_xyzw(rotation_matrix):
    r = R_scipy.from_matrix(rotation_matrix)
    qx, qy, qz, qw = r.as_quat()
    return qx, qy, qz, qw

def rotmat_to_rpy_deg(rotation_matrix):
    r = R_scipy.from_matrix(rotation_matrix)
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
    return roll, pitch, yaw

def save_pose_and_rpy(current_time, position_xyz, R_world_tag, camera_name):
    qx, qy, qz, qw = rotmat_to_quat_xyzw(R_world_tag)
    roll_deg, pitch_deg, yaw_deg = rotmat_to_rpy_deg(R_world_tag)
    row = {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "camera": camera_name,
        "pos_x": round(float(position_xyz[0]), 4),
        "pos_y": round(float(position_xyz[1]), 4),
        "pos_z": round(float(position_xyz[2]), 4),
        "quat_x": round(float(qx), 4),
        "quat_y": round(float(qy), 4),
        "quat_z": round(float(qz), 4),
        "quat_w": round(float(qw), 4),
        "roll_deg": round(float(roll_deg)),
        "pitch_deg": round(float(pitch_deg)),
        "yaw_deg": round(float(yaw_deg)),
        "vel_x": 0.0,
        "vel_y": 0.0,
        "vel_z": 0.0,
        "vel_x_avg5": 0.0,
        "vel_x_ema": 0.0,
    }
    return row

def process_pose_T(matrix, state, point, current_time, camera_name):
    if matrix is None or matrix.shape != (4, 4):
        return point

    point = matrix[:3, 3].astype(np.float64)
    rotation_matrix = matrix[:3, :3].astype(np.float64)

    if not is_valid_rotation_matrix(rotation_matrix):
        return point

    # Calcular velocidad cruda
    vx = vy = vz = 0.0
    old_p = state.get("old_point")
    old_t = state.get("old_time")
    if old_p is not None and old_t is not None:
        dt = current_time - old_t
        if dt >= 0.01:
            v = (point - old_p) / dt
            vx, vy, vz = v.tolist()

    # --- Average 5 frames en vel_x ---
    vel_x_buffer = state.get("vel_x_buffer", deque(maxlen=5))
    vel_x_buffer.append(vx)
    state["vel_x_buffer"] = vel_x_buffer
    vel_x_avg5 = float(np.mean(vel_x_buffer))

    # --- EMA en vel_x con rechazo de outliers ---
    EMA_ALPHA = 0.3
    MAX_VEL = 0.10  # m/s maximo esperado
    old_vx_ema = state.get("vel_x_ema", 0.0)
    vx_raw = vx if abs(vx) <= MAX_VEL else old_vx_ema
    vel_x_ema = EMA_ALPHA * vx_raw + (1 - EMA_ALPHA) * old_vx_ema
    state["vel_x_ema"] = vel_x_ema

    row = save_pose_and_rpy(current_time, point, rotation_matrix, camera_name)
    row["vel_x"] = round(float(vx), 4)
    row["vel_y"] = round(float(vy), 4)
    row["vel_z"] = round(float(vz), 4)
    row["vel_x_avg5"] = round(float(vel_x_avg5), 4)
    row["vel_x_ema"]  = round(float(vel_x_ema), 4)

    with camera_rows_lock:
        camera_rows.append(row)

    now_t = time.monotonic()
    if now_t - last_print_time.get(camera_name, 0) >= PRINT_INTERVAL:
        last_print_time[camera_name] = now_t
        print(
            f"[{camera_name}] "
            f"pos=({row['pos_x']:.3f}, {row['pos_y']:.3f}, {row['pos_z']:.3f}) m | "
            f"rot=({row['roll_deg']}°, {row['pitch_deg']}°, {row['yaw_deg']}°) | "
            f"vel_x={row['vel_x']:.3f} | avg5={row['vel_x_avg5']:.3f} | ema={row['vel_x_ema']:.3f} m/s"
        )

    state["old_point"] = point.copy()
    state["old_time"] = current_time
    return point

def pose_to_matrix(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation.reshape(3)
    return matrix

def draw_tag_overlay(frame, det):
    corners = det.corners.astype(int)
    for i in range(4):
        p0 = tuple(corners[i])
        p1 = tuple(corners[(i + 1) % 4])
        cv2.line(frame, p0, p1, (0, 255, 0), 2)
    c = tuple(det.center.astype(int))
    cv2.circle(frame, c, 4, (0, 0, 255), -1)
    return frame

def lost_track_tag(state):
    if state.get("old_point") is not None:
        state["old_point"] = None
        state["old_time"] = None
        state["vel_x_ema"] = 0.0
        state["vel_x_buffer"] = deque(maxlen=5)
    return None

#####################################################################################################
########### Hilo para leer el Arduino por serial en paralelo

def read_arduino(port, baud, stop_event):
    global current_pair_time
    try:
        ser = pyserial.Serial(port, baud, timeout=1)
        print(f"[Arduino] Conectado en {port} a {baud} baud")
    except Exception as e:
        print(f"[Arduino] No se pudo conectar: {e}")
        print(f"[Arduino] Continuando sin datos de Arduino...")
        return

    header_skipped = False
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8").strip()
            if not line:
                continue
            if not header_skipped:
                header_skipped = True
                continue
            parts = line.split(",")
            if len(parts) != 8:
                continue
            row = {
                "timestamp_ms": int(parts[0]),
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                "rpm":     float(parts[1]),
                "accel_x": float(parts[2]),
                "accel_y": float(parts[3]),
                "accel_z": float(parts[4]),
                "gyro_x":  float(parts[5]),
                "gyro_y":  float(parts[6]),
                "gyro_z":  float(parts[7]),
            }
            with arduino_rows_lock:
                arduino_rows.append(row)
        except Exception:
            continue

    ser.close()
    print("[Arduino] Conexion cerrada.")

#####################################################################################################
########### Funcion para convertir frame de profundidad a escala de grises

def depth_to_gray(depth_frame):
    depth_img = np.asanyarray(depth_frame.get_data()).astype(np.float32)
    valid_mask = depth_img > 0
    if valid_mask.any():
        min_val = depth_img[valid_mask].min()
        max_val = depth_img[valid_mask].max()
        if max_val > min_val:
            depth_img[valid_mask] = (depth_img[valid_mask] - min_val) / (max_val - min_val) * 255.0
    depth_gray = depth_img.astype(np.uint8)
    return cv2.cvtColor(depth_gray, cv2.COLOR_GRAY2BGR)

#####################################################################################################
########### Funcion para guardar todo en Excel al finalizar

def save_to_excel(filename):
    wb = openpyxl.Workbook()

    ws_cam = wb.active
    ws_cam.title = "Camaras"
    cam_headers = [
        "timestamp", "camera",
        "pos_x", "pos_y", "pos_z",
        "quat_x", "quat_y", "quat_z", "quat_w",
        "roll_deg", "pitch_deg", "yaw_deg",
        "vel_x", "vel_y", "vel_z",
        "vel_x_avg5", "vel_x_ema"
    ]
    ws_cam.append(cam_headers)
    with camera_rows_lock:
        for row in camera_rows:
            ws_cam.append([row[h] for h in cam_headers])

    ws_ard = wb.create_sheet(title="Arduino")
    ard_headers = [
        "timestamp_ms", "timestamp", "rpm",
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z"
    ]
    ws_ard.append(ard_headers)
    with arduino_rows_lock:
        for row in arduino_rows:
            ws_ard.append([row[h] for h in ard_headers])

    wb.save(filename)
    print(f"\n[Excel] Datos guardados en: {filename}")
    print(f"  - Filas de camara:  {len(camera_rows)}")
    print(f"  - Filas de Arduino: {len(arduino_rows)}")

#####################################################################################################
########### Funciones de sincronizacion de camaras

def poll_latest(pipeline, max_drain=10):
    chosen_frame = None
    for _ in range(max_drain):
        frameset = pipeline.poll_for_frames()
        if not frameset:
            break
        chosen_frame = frameset
    return chosen_frame

def estimate_host_ts_ms(frameset, cam_state, beta=0.02):
    colour_frame = frameset.get_color_frame()
    if not colour_frame:
        return None
    dev_ms = colour_frame.get_timestamp()
    dev_s  = dev_ms * 1e-3
    host_s = time.monotonic()
    time_diff = host_s - dev_s
    if cam_state.get("offset_s") is None:
        cam_state["offset_s"] = time_diff
    else:
        cam_state["offset_s"] = (1.0 - beta) * cam_state["offset_s"] + beta * time_diff
    est_host_s = dev_s + cam_state["offset_s"]
    return est_host_s * 1000.0

class FrameSync:
    def __init__(self, max_time_diff=15.0, buffer_ms=100.0):
        self.max_time_diff = float(max_time_diff)
        self.buffer_ms = float(buffer_ms)
        self.buffer_master = deque()
        self.buffer_slave  = deque()

    def push_master(self, frameset, host_timestamp_ms):
        self._push(self.buffer_master, frameset, host_timestamp_ms)
    def push_slave(self, frameset, host_timestamp_ms):
        self._push(self.buffer_slave, frameset, host_timestamp_ms)

    def _push(self, buffer, frameset, timestamp_ms):
        buffer.append((float(timestamp_ms), frameset))
        self._trim_old(buffer, float(timestamp_ms))

    def _trim_old(self, buffer, newest_timestamp):
        cutoff = newest_timestamp - self.buffer_ms
        while buffer and buffer[0][0] < cutoff:
            buffer.popleft()

    def try_match(self):
        if not self.buffer_master or not self.buffer_slave:
            return None, None, None, None
        master_timestamp, master_frameset = self.buffer_master[-1]
        best_i = None
        best_dt = None
        for i, (slave_timestamp, slave_frameset) in enumerate(self.buffer_slave):
            time_delta = abs(master_timestamp - slave_timestamp)
            if best_dt is None or time_delta < best_dt:
                best_dt = time_delta
                best_i = i
        if best_dt is None or best_dt > self.max_time_diff:
            while len(self.buffer_master) > 1:
                self.buffer_master.popleft()
            cutoff = master_timestamp - self.max_time_diff
            while self.buffer_slave and self.buffer_slave[0][0] < cutoff:
                self.buffer_slave.popleft()
            return None, None, None, None
        slave_timestamp, slave_frameset = self.buffer_slave[best_i]
        for _ in range(best_i + 1):
            self.buffer_slave.popleft()
        self.buffer_master.clear()
        return master_timestamp, master_frameset, slave_frameset, best_dt

#####################################################################################################
########### Inicializar camaras Realsense

realsense_context = rs.context()
devices = realsense_context.query_devices()

if len(devices) < 1:
    raise RuntimeError("No RealSense cameras found")

serials = []
for d in devices:
    serials.append(d.get_info(rs.camera_info.serial_number))
print("Found cameras:", serials)

serial_master = serials[0]
serial_slaves = serials[1:]

pipeline_master = rs.pipeline()
config_master = rs.config()
config_master.enable_device(serial_master)
config_master.enable_stream(rs.stream.color, 448, 252, rs.format.bgr8, FRAME_RATE)
config_master.enable_stream(rs.stream.depth, 448, 252, rs.format.z16, FRAME_RATE)
pipeline_master.start(config_master)

frames = pipeline_master.wait_for_frames()
c = frames.get_color_frame()
intr = c.profile.as_video_stream_profile().intrinsics
camera_params_master = (intr.fx, intr.fy, intr.ppx, intr.ppy)
align_master = rs.align(rs.stream.color)

master_state = {"old_point": None, "old_time": None, "offset_s": None}
master_point = None

video_master_color = cv2.VideoWriter(f"master_color_{timestamp_str}.avi", VIDEO_CODEC, FRAME_RATE, (448, 252))
video_master_depth = cv2.VideoWriter(f"master_depth_{timestamp_str}.avi", VIDEO_CODEC, FRAME_RATE, (448, 252))
print(f"[Video] Grabando → master_color_{timestamp_str}.avi")
print(f"[Video] Grabando → master_depth_{timestamp_str}.avi")

slaves = []
for i, serial in enumerate(serial_slaves):
    pipeline_slave = rs.pipeline()
    config_slave = rs.config()
    config_slave.enable_device(serial)
    config_slave.enable_stream(rs.stream.color, 448, 252, rs.format.bgr8, FRAME_RATE)
    config_slave.enable_stream(rs.stream.depth, 448, 252, rs.format.z16, FRAME_RATE)
    pipeline_slave.start(config_slave)

    frames = pipeline_slave.wait_for_frames()
    c = frames.get_color_frame()
    intr = c.profile.as_video_stream_profile().intrinsics
    camera_params_slave = (intr.fx, intr.fy, intr.ppx, intr.ppy)

    slaves.append({
        "serial": serial,
        "pipeline": pipeline_slave,
        "sync": FrameSync(),
        "point": None,
        "camera_params": camera_params_slave,
        "state": {"old_point": None, "old_time": None, "offset_s": None},
        "name": f"cam_slave_{i + 1}",
        "window": f"Video Slave {i + 1}",
        "align": rs.align(rs.stream.color),
        "video_color": cv2.VideoWriter(f"slave_{i+1}_color_{timestamp_str}.avi", VIDEO_CODEC, FRAME_RATE, (448, 252)),
        "video_depth": cv2.VideoWriter(f"slave_{i+1}_depth_{timestamp_str}.avi", VIDEO_CODEC, FRAME_RATE, (448, 252)),
    })

#####################################################################################################
########### Arrancar hilo del Arduino

stop_event = threading.Event()
arduino_thread = threading.Thread(
    target=read_arduino,
    args=(ARDUINO_PORT, ARDUINO_BAUD, stop_event),
    daemon=True
)
arduino_thread.start()

#####################################################################################################
########### Loop principal

print("Corriendo... Presiona ESC para detener y guardar Excel.")

try:
    while True:

        frameset_master = poll_latest(pipeline_master, max_drain=30)
        if not frameset_master:
            continue

        frameset_master = align_master.process(frameset_master)
        timestamp_master_ms = estimate_host_ts_ms(frameset_master, master_state)
        if timestamp_master_ms is None:
            continue

        m_frames = frameset_master

        for slave in slaves:
            slave["sync"].push_master(m_frames, timestamp_master_ms)

        matches = []
        all_ok = True

        for slave in slaves:
            frameset_slave = poll_latest(slave["pipeline"], max_drain=30)
            if frameset_slave:
                frameset_slave = slave["align"].process(frameset_slave)
                timestamp_slave_ms = estimate_host_ts_ms(frameset_slave, slave["state"])
                if timestamp_slave_ms is not None:
                    slave["sync"].push_slave(frameset_slave, timestamp_slave_ms)

            master_timestamp, master_frameset, slave_frameset, time_delta = slave["sync"].try_match()
            if master_frameset is None:
                all_ok = False
                break

            matches.append((slave, master_timestamp, master_frameset, slave_frameset, time_delta))

        if not all_ok:
            continue

        if len(matches) == 0:
            pair_time = time.monotonic()
            m_pair = frameset_master
        else:
            pair_time = matches[0][1] * 1e-3
            m_pair = matches[0][2]

        current_pair_time = pair_time

        color = m_pair.get_color_frame()
        depth = m_pair.get_depth_frame()
        if not color or not depth:
            continue

        color_img = np.asanyarray(color.get_data())
        depth_img = depth_to_gray(depth)
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        world_detections = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params_master, tag_size=world_tag_size)
        target_detections = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params_master, tag_size=target_tag_size)

        overlay = color_img.copy()
        world_det  = None
        target_det = None
        for d in world_detections:
            if d.tag_id == WORLD_TAG_ID:
                world_det = d
        for d in target_detections:
            if d.tag_id == TRACK_TAG_ID:
                target_det = d

        if target_det is None or world_det is None:
            master_point = lost_track_tag(master_state)
        else:
            overlay = draw_tag_overlay(overlay, target_det)
            overlay = draw_tag_overlay(overlay, world_det)
            T_cam_world = pose_to_matrix(world_det.pose_R, world_det.pose_t)
            T_cam_tag   = pose_to_matrix(target_det.pose_R, target_det.pose_t)
            T_world_cam = np.linalg.inv(T_cam_world)
            T_world_tag = T_world_cam @ T_cam_tag
            master_point = process_pose_T(T_world_tag, master_state, master_point, pair_time, "cam_master")

        cv2.imshow("Master - Color", overlay)
        cv2.imshow("Master - Depth", depth_img)
        video_master_color.write(overlay)
        video_master_depth.write(depth_img)

        for (slave, master_timestamp, master_frameset, slave_frameset, time_delta) in matches:
            color = slave_frameset.get_color_frame()
            depth = slave_frameset.get_depth_frame()
            if not color or not depth:
                continue

            color_img = np.asanyarray(color.get_data())
            depth_img = depth_to_gray(depth)
            gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

            world_detections  = detector.detect(gray, estimate_tag_pose=True, camera_params=slave["camera_params"], tag_size=world_tag_size)
            target_detections = detector.detect(gray, estimate_tag_pose=True, camera_params=slave["camera_params"], tag_size=target_tag_size)

            overlay = color_img.copy()
            world_det  = None
            target_det = None
            for d in world_detections:
                if d.tag_id == WORLD_TAG_ID:
                    world_det = d
            for d in target_detections:
                if d.tag_id == TRACK_TAG_ID:
                    target_det = d

            if target_det is None or world_det is None:
                slave["point"] = lost_track_tag(slave["state"])
            else:
                overlay = draw_tag_overlay(overlay, target_det)
                overlay = draw_tag_overlay(overlay, world_det)
                T_cam_world = pose_to_matrix(world_det.pose_R, world_det.pose_t)
                T_cam_tag   = pose_to_matrix(target_det.pose_R, target_det.pose_t)
                T_world_cam = np.linalg.inv(T_cam_world)
                T_world_tag = T_world_cam @ T_cam_tag
                slave["point"] = process_pose_T(T_world_tag, slave["state"], slave["point"], pair_time, slave["name"])

            cv2.imshow(slave["window"] + " - Color", overlay)
            cv2.imshow(slave["window"] + " - Depth", depth_img)
            slave["video_color"].write(overlay)
            slave["video_depth"].write(depth_img)

        if cv2.waitKey(2) & 0xFF == 27:
            break

finally:
    stop_event.set()
    pipeline_master.stop()
    for slave in slaves:
        slave["pipeline"].stop()

    video_master_color.release()
    video_master_depth.release()
    for slave in slaves:
        slave["video_color"].release()
        slave["video_depth"].release()
    print("[Video] Archivos de video guardados.")

    cv2.destroyAllWindows()
    save_to_excel(EXCEL_FILENAME)