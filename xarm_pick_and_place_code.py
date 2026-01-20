#!/usr/bin/env python3

import os, time, argparse
import numpy as np
import cv2
from xarm.wrapper import XArmAPI

#Manually correct the Y-axis by increasing it if the grasp is too far right, decreasing it if it is too far left
Y_shift = 10.0 

# Safe vertical lift after grasping to avoid collision
safe_Z_lift = 130.0 

# Calibration
# Rotation from camera frame to robot base frame
R_cam_to_robot = np.array([
    [0.99938490, 0.00237150, -0.03498840],
    [0.03089160, -0.53177294, 0.84632337],
    [-0.01659883, -0.84688365, -0.53151911]
], dtype=float)

# Translation from camera origin to robot base 
t_cam_to_robot = np.array([
    175.40945600,
    -430.32757326,
    837.47972000
], dtype=float)

# Environment parameters
table_Z = 56.0    # table height
bottom_scan_ratio = 0.90    # sample point near bottom of bounding box
Z_offset = 27.7    # height compensation for object thickness

# Position refinement
#It is a static calibration model and should be retuned if the camera, table, or setup changes.
# X_est, Y_est: Raw X and Y coordinates computed from the camera
# X_corr, Y_corr : Corrected robot target coordinates

def refine_xy(X_est, Y_est):
    X_corr = 0.932703 * X_est + (-0.116488) * Y_est + 68.8144
    
    # Orijinal offset: 79.6724
    Y_corr = -0.125401 * X_est + 0.773156  * Y_est + (79.6724 + Y_shift)
    
    return X_corr, Y_corr

# Load the YOLO model
def load_yolo(cfg, weights, names, use_cuda=False):
    # Implements a vision-guided pick-and-place pipeline using YOLO, RealSense, and xArm.
    # RealSense camera initialization and RGB frame capture
    with open(names, "r") as f:
        classes = [line.strip() for line in f]
    lemon_id = classes.index("lemon")
    net = cv2.dnn.readNetFromDarknet(cfg, weights)
    if use_cuda:
        try:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        except:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    else:
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    ln = net.getLayerNames()
    out_layers = [ln[i - 1] for i in net.getUnconnectedOutLayers().flatten()]
    return net, out_layers, lemon_id

# Implementing a vision-guided pick-and-place pipeline using YOLO, RealSense, and xArm.
def init_realsense():
    # RealSense camera initialization and RGB frame capture
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.disable_all_streams()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(cfg)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    return rs, pipeline, intr

def get_color_frame(rs, pipeline):
    try:
        frames = pipeline.wait_for_frames(10000)
        return frames.get_color_frame()
    except:
        return None

# Detection and Pixel-to-Robot projection
def detect_lemon(net, out_layers, img, lemon_id, conf=0.35, nms=0.45):
    # Runs YOLO inference on the given image and returns the lemon detection.
    # Filters detections by class "lemon" and confidence threshold, 
    # and chooses the largest detected lemon (by bounding-box) as the target
    H, W = img.shape[:2]
    blob = cv2.dnn.blobFromImage(img, 1/255.0, (416,416), swapRB=True, crop=False)
    net.setInput(blob)
    outs = net.forward(out_layers)
    boxes = []
    confs = []
    for out in outs:
        for det in out:
            scores = det[5:]
            cls = int(np.argmax(scores))
            c = float(scores[cls])
            if cls != lemon_id or c < conf:
                continue
            cx, cy, w, h = det[0]*W, det[1]*H, det[2]*W, det[3]*H
            x = int(cx - w/2)
            y = int(cy - h/2)
            boxes.append([x, y, int(w), int(h)])
            confs.append(c)
    if not boxes:
        return None, None, None
    i = np.argmax([w*h for (_,_,w,h) in boxes])
    x, y, w, h = boxes[i]
    return (x+w/2, y+h/2), (x, y, w, h), confs[i]

def get_bottom_pixel(x, y, w, h):
    # Returns a pixel near the bottom of the bounding box to approximate the contact point with the table.
    return int(x + w/2), int(y + h*bottom_scan_ratio)

def pixel_to_robot(u, v, intr):
    # Projects a pixel (u, v) into the robot base frame by intersecting the camera ray with the table plane
    xn = (u - intr.ppx) / intr.fx
    yn = (v - intr.ppy) / intr.fy
    ray_cam = np.array([xn, yn, 1.0])
    ray_cam /= np.linalg.norm(ray_cam)
    ray_robot = R_cam_to_robot @ ray_cam
    origin = t_cam_to_robot
    vz = ray_robot[2]
    if abs(vz) < 1e-6:
        return None
    alpha = (table_Z - origin[2]) / vz
    if alpha <= 0:
        return None
    P = origin + alpha * ray_robot
    return P

# XARM robot control
robot_IP = "192.168.1.225"
move_speed = 100  #motion parameters
move_acc = 1000
gripper_open = 650   #gripper positions
gripper_close = 630
gripper_speed = 500
 
bowl_X = 198.5   # predefined drop location (bowl center) in robot base frame
bowl_Y = 374.6
bowl_Z = 85.1

# Approach height above the bowl to avoid collisions during drop
drop_approach_Z = bowl_Z + 80.0

def prepare_robot():
    # Connects to the xArm, enables motion, and initializes the gripper. 
    # Returns an XArmAPI instance ready for position commands.
    arm = XArmAPI(robot_IP, is_radian=False)
    print("[robot] Initialized")
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_gripper_enable(True)
    arm.set_gripper_mode(0)
    arm.set_gripper_speed(gripper_speed)
    return arm

def safe_move(arm, x, y, z): 
    # Executes a synchronous cartesian move with fixed roll/pitch/yaw for consistent grasping.
    code = arm.set_position(
        x=x, y=y, z=z,
        roll=180, pitch=0, yaw=90,
        speed=move_speed, mvacc=move_acc,
        wait=True
    )
    return code

def gripper_open_cmd(arm):  
    # Opens the gripper to the predefined open position.  
    print("[gripper] Opening")
    arm.set_gripper_position(gripper_open, wait=True)

def gripper_close_cmd(arm):  
    # Closes the gripper to the predefined close position (grasp).
    print("[gripper] Closing")
    arm.set_gripper_position(gripper_close, wait=True)

def pick_lemon(arm, X, Y, Z): 
    # To approach the target from above, grasp the object, and lift it to a safe height.
    print(f"[Pick] target = ({X:.1f}, {Y:.1f}, {Z:.1f}) | Y-Shift: {Y_shift}mm applied")
    safe_move(arm, X, Y, Z+80)
    safe_move(arm, X, Y, Z)
    gripper_close_cmd(arm)
    time.sleep(0.5)
    print(f"[move up] Safe height: {Z+safe_Z_lift}")
    safe_move(arm, X, Y, Z+safe_Z_lift)

def drop_to_bowl(arm):
    # Drops the lemon into the bowl by first moving to a safe approach height above the bowl,
    # then descending to the drop position, opening the gripper, and finally retracting upward to prevent collisions.
    print(f"[drop] Bowl = ({bowl_X}, {bowl_Y}, {bowl_Z})")
    safe_move(arm, bowl_X, bowl_Y, drop_approach_Z)
    safe_move(arm, bowl_X, bowl_Y, bowl_Z)
    gripper_open_cmd(arm)
    safe_move(arm, bowl_X, bowl_Y, drop_approach_Z)

def main():
    # Main loop
    # Initializes YOLO, RealSense, and xArm, then persistently detects for a lemon in the RGB stream.
    # When a valid 3D target is estimated, pressing '.p' triggers a pick-and-place action. 
    args = parse_args()
    net, out_layers, lemon_id = load_yolo(args.cfg, args.weights, args.names, args.cuda)
    rs, pipeline, intr = init_realsense()
    arm = prepare_robot()
    print("\n If lemon detected, press 'p' ---\n")
    X = Y = Z = None
    world_valid = False

    while True:
        color = get_color_frame(rs, pipeline) # Acquire latest RGB frame
        if not color: continue
        img = np.asanyarray(color.get_data())
        # Detect lemon and calculate its bounding box
        center, box, conf = detect_lemon(net, out_layers, img, lemon_id, args.conf, args.nms)
        vis = img.copy()
        world_valid = False

        if center and box:
            x, y, w, h = box
            u, v = get_bottom_pixel(x, y, w, h)
           # Draw bounding box and projected grasp point for visualization
            cv2.rectangle(vis, (x,y), (x+w,y+h), (0,255,0), 2)
            cv2.circle(vis, (u,v), 5, (0,0,255), -1)
           # Convert image pixel to robot coordinates using ray–table intersection
            P = pixel_to_robot(u, v, intr)
            if P is not None:
                X_raw, Y_raw, Z_raw = P
               #Employ the Z grasp offset and XY correction.
                X, Y = refine_xy(X_raw, Y_raw) 
                Z = Z_raw + Z_offset
                world_valid = True
                cv2.putText(vis, f"Ready Y+{Y_shift}mm ({X:.0f},{Y:.0f})", (10,30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        cv2.imshow("YOLO + xArm", vis)

        # Key controls: 'p' pick&place, 'd' drop, 'o' open, 'c' close, 'q'/ESC exit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('p') and world_valid:
            pick_lemon(arm, X, Y, Z)
            drop_to_bowl(arm)
            print("Pick-and-place completed.")
        elif key == ord('d'): drop_to_bowl(arm)
        elif key == ord('o'): gripper_open_cmd(arm)
        elif key == ord('c'): gripper_close_cmd(arm)
        elif key in (27, ord('q')): break

    pipeline.stop()
    cv2.destroyAllWindows()

def parse_args():
    # Parses command-line arguments for YOLO model paths, detection thresholds,
    # and optional CUDA acceleration.
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default="/home/emre/Downloads/yolov4-colors.cfg")
    ap.add_argument("--weights", default="/home/emre/Downloads/yolov4-colors.weights")
    ap.add_argument("--names", default="/home/emre/Downloads/colors.names")
    ap.add_argument("--nms", type=float, default=0.45)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--cuda", action="store_true")
    return ap.parse_args()

if __name__ == "__main__":
    main()
