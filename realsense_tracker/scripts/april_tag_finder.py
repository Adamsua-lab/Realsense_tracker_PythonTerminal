from argparse import ArgumentParser
import os
import cv2
import numpy as np
from pupil_apriltags import Detector
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 448, 252, rs.format.bgr8, 60) # colour stream
config.enable_stream(rs.stream.depth, 448, 252, rs.format.z16, 60) # depth stream

profile = pipeline.start(config)

align = rs.align(rs.stream.color)

################################################################################

def draw_tag_overlay(frame, det, camera_params, tag_size_m, draw_axes=True):
    """
    Draw AprilTag bounding quad, center, ID, and optional pose axes.
    det: pupil_apriltags detection
    """
    fx, fy, cx, cy = camera_params

    # Draw tag outline
    corners = det.corners.astype(int)  # shape (4,2)
    for i in range(4):
        p0 = tuple(corners[i])
        p1 = tuple(corners[(i + 1) % 4])
        cv2.line(frame, p0, p1, (0, 255, 0), 2)

    # Draw center + ID
    c = tuple(det.center.astype(int))
    cv2.circle(frame, c, 4, (0, 0, 255), -1)
    cv2.putText(frame, f"id={det.tag_id}", (c[0] + 10, c[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Draw pose axes if pose was computed
    if draw_axes and det.pose_R is not None and det.pose_t is not None:
        # Build projection
        K = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0,  0,  1]], dtype=np.float64)

        R = det.pose_R.astype(np.float64)
        t = det.pose_t.astype(np.float64).reshape(3, 1)

        # Define axes endpoints in tag frame (meters)
        axis_len = tag_size_m * 0.5
        pts_3d = np.array([
            [0, 0, 0],
            [axis_len, 0, 0],  # X (red)
            [0, axis_len, 0],  # Y (green)
            [0, 0, -axis_len], # Z (blue) (camera looks down +Z, tag coords vary; this is a reasonable viz)
        ], dtype=np.float64)

        # Project 3D to 2D: X_cam = R*X_tag + t
        pts_cam = (R @ pts_3d.T) + t  # 3xN
        pts_img = (K @ pts_cam)       # 3xN
        pts_img = (pts_img[:2, :] / pts_img[2:3, :]).T  # Nx2

        p0 = tuple(pts_img[0].astype(int))
        px = tuple(pts_img[1].astype(int))
        py = tuple(pts_img[2].astype(int))
        pz = tuple(pts_img[3].astype(int))

        cv2.line(frame, p0, px, (0, 0, 255), 2)   # X red
        cv2.line(frame, p0, py, (0, 255, 0), 2)   # Y green
        cv2.line(frame, p0, pz, (255, 0, 0), 2)   # Z blue

    return frame

################################################################################

def apriltag_video(
    input_streams=('single_tag.mp4'),  # For default cam use -> [0]
    output_stream=False,
    display_stream=True,
    detection_window_name='AprilTag',
):

    # If user passed a single stream as a string/int, wrap it into a list
    if isinstance(input_streams, (str, int)):
        input_streams = [input_streams]
        
    parser = ArgumentParser(description='Detect AprilTags from video stream (pupil-apriltags).')
    parser.add_argument("--family", default="tag16h5", help="AprilTag family (e.g., tag16h5, tag25h9)")
    parser.add_argument("--nthreads", type=int, default=2)
    parser.add_argument("--quad_decimate", type=float, default=1.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", action="store_true")
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--min_hamming", type=int, default=0)
    args = parser.parse_args()

    # Your intrinsics + tag size
    camera_params = (3156.71852, 3129.52243, 359.097908, 239.736909)  # fx, fy, cx, cy
    tag_size = 0.15  # meters

    detector_kwargs = dict(
    families=args.family,
    nthreads=args.nthreads,
    quad_decimate=args.quad_decimate,
    quad_sigma=args.quad_sigma,
    refine_edges=args.refine_edges,
    decode_sharpening=args.decode_sharpening,
)

    # Some versions don't support extra args like min_hamming; only pass what exists
    detector = Detector(**detector_kwargs)

    while True:
        frame = align.process(pipeline.wait_for_frames())
        colour_frame = frame.get_color_frame()
        frame = np.asanyarray(colour_frame.get_data())

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # estimate_tag_pose=True gives pose_R, pose_t
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size
        )

        overlay = frame.copy()
        for det in detections:
            overlay = draw_tag_overlay(overlay, det, camera_params, tag_size_m=tag_size, draw_axes=True)

        if display_stream:
            cv2.imshow(detection_window_name, overlay)
            if (cv2.waitKey(1) & 0xFF) == ord(' '):  # space to quit
                break

    if display_stream:
        cv2.destroyAllWindows()

################################################################################

if __name__ == '__main__':
    apriltag_video()
