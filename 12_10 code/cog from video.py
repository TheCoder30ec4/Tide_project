import cv2
import mediapipe as mp

# Initialize MediaPipe Pose
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

# Open the video file
video_path = "D:/Nithin/STUDIES/FALL DEETECTYION/code/output/test_video.mp4"
cap = cv2.VideoCapture(video_path)

pTime = 0

while True:
    ret, color_image = cap.read()

    if not ret:
        break

    # MediaPipe Pose on RGB frames
    imgRGB = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = pose.process(imgRGB)

    if results.pose_landmarks:
        # Get landmarks for the hips (landmarks 23 and 24)
        hip_landmarks = [results.pose_landmarks.landmark[23], results.pose_landmarks.landmark[24]
        ]
        
        # Calculate Center of Gravity (CoG) between the hips
        cog_x = int((hip_landmarks[0].x + hip_landmarks[1].x) * color_image.shape[1] / 2)
        cog_y = int((hip_landmarks[0].y + hip_landmarks[1].y) * color_image.shape[0] / 2) + 200

        # Draw CoG on the image
        cv2.circle(color_image, (cog_x, cog_y), 10, (255, 255, 255), -1)

        # Draw pose landmarks on the image
        mpDraw.draw_landmarks(color_image, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

    cTime = cv2.getTickCount()
    fps = 1 / ((cTime - pTime) / cv2.getTickFrequency())
    pTime = cTime
    cv2.putText(color_image, f"FPS: {int(fps)}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    # Display the RGB image with Pose landmarks
    cv2.imshow("RGB Image with Pose Landmarks", color_image)

    if cv2.waitKey(1) == ord('q'):
        break

# Release the video file and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
