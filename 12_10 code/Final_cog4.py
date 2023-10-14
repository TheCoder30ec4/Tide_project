import cv2
import mediapipe as mp
import time
import pyrealsense2 as rs
import numpy as np

# Initialize MediaPipe Pose
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

# Initialize RealSense Pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)
depth_image_colormap = np.zeros((480, 640, 3), dtype=np.uint8)

pTime = 0

while True:
    # RealSense Camera
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # MediaPipe Pose on RGB frames
    imgRGB = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = pose.process(imgRGB)

    if results.pose_landmarks:
        # Get landmarks for the hips (landmarks 23 and 24)
        hip_landmarks = [results.pose_landmarks.landmark[23], results.pose_landmarks.landmark[24]]
        
        # Calculate Center of Gravity (CoG) between the hips
        cog_x = int((hip_landmarks[0].x + hip_landmarks[1].x) / 2 * color_image.shape[1])
        cog_y = int((hip_landmarks[0].y + hip_landmarks[1].y) / 2 * color_image.shape[0])+ 120

        # Draw CoG on the image
        cv2.circle(color_image, (cog_x, cog_y), 10, (0, 255, 0), -1)

        # Convert depth image to BGR for drawing landmarks
        depth_image_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)
        mpDraw.draw_landmarks(depth_image_colormap, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime
    cv2.putText(color_image, f"FPS: {int(fps)}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    # Display the depth image with pose landmarks and colormap
    cv2.imshow("Depth Image with Pose Landmarks", depth_image_colormap)
    cv2.imshow("RGB Image with Pose Landmarks", color_image)

    if cv2.waitKey(1) == ord('q'):
        break

# Cleanup
pipe.stop()
cv2.destroyAllWindows()
