import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
base_options = python.BaseOptions(model_asset_path='MediaPipe/pose_landmarker_heavy.task')
# options = vision.PoseLandmarkerOptions(
#     base_options=base_options,
#     output_segmentation_masks=True)
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO)
detector = vision.PoseLandmarker.create_from_options(options)

from mediapipe.python.solutions.pose import PoseLandmark
my_drawing_style = solutions.drawing_styles.get_default_pose_landmarks_style()
from mediapipe.python.solutions.drawing_utils import DrawingSpec
# Customize the drawing styles
# Change joint sizes
my_drawing_style[PoseLandmark.LEFT_WRIST].circle_radius = 20
my_drawing_style[PoseLandmark.LEFT_WRIST].thickness = 10
my_drawing_style[PoseLandmark.RIGHT_WRIST].circle_radius = 20
my_drawing_style[PoseLandmark.RIGHT_WRIST].thickness = 10
# Change connection thickness

def draw_landmarks_on_image(rgb_image, detection_result):
    pose_landmarks_list = detection_result.pose_landmarks
    annotated_image = np.copy(rgb_image)

    # Loop through the detected poses to visualize.
    for idx in range(len(pose_landmarks_list)):
        pose_landmarks = pose_landmarks_list[idx]

        # Draw the pose landmarks.
        pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        pose_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
        ])
        solutions.drawing_utils.draw_landmarks(
            annotated_image,
            pose_landmarks_proto,
            solutions.pose.POSE_CONNECTIONS,
            my_drawing_style
            )
    return annotated_image

import cv2
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'functions')))
from load_machine_config import load_machine_config

video_name = load_machine_config()['data_dir'] + 'Motorized_Camera/' + 'figures/' + 'VR_sit.mov'

cap = cv2.VideoCapture(video_name)

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
frame_rate = int(cap.get(5))
num_frames = int(cap.get(7))

vid = cv2.VideoWriter(load_machine_config()['data_dir'] + 'Motorized_Camera/' + 'figures/' + 'annotated_output.mp4', 
                      cv2.VideoWriter_fourcc(*'mp4v'), 
                        frame_rate,
                        (frame_width, frame_height))

print("Frame width: ", frame_width)
print("Frame height: ", frame_height)
print("Frame rate: ", frame_rate)
print("Number of frames: ", num_frames)

frame_count = 0
with vision.PoseLandmarker.create_from_options(options) as landmarker:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Video frames finished.")
            break
        frame_count += 1
        print("Frames:", str(frame_count)+"/" + str(num_frames), end="\r")
        frame_timestamp_ms = int(cap.get(cv2.CAP_PROP_POS_MSEC))

        # image_720p = cv2.resize(image, (1280, 720))
        image_to_process = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        detection_result = landmarker.detect_for_video(image_to_process, frame_timestamp_ms)
        # detection_result = detector.detect(image_to_process)

        annotated_image = draw_landmarks_on_image(image_to_process.numpy_view(), detection_result)
        # cv2.imshow("Annotated Image", annotated_image)
        if cv2.waitKey(5) & 0xFF == 27: # ESC key to exit
            break

        # Save the annotated video
        vid.write(annotated_image)
vid.release()
cap.release()
cv2.destroyAllWindows() 

    