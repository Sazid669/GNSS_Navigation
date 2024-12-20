#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os
from natsort import natsorted  # For sorting files naturally

# Directory containing image frames
image_dir = 'yolo_output'  # Change this to your directory path
output_video_path = 'real_robot_output_video.avi'  # Output video file

# Video settings
frame_rate = 30  # FPS of the video
frame_size = None  # Will be determined based on the first image

# Collect all image file paths
image_files = [os.path.join(image_dir, f) for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png'))]
image_files = natsorted(image_files)  # Sort files naturally to maintain the frame order

if not image_files:
    raise ValueError(f"No image frames found in the directory: {image_dir}")

# Initialize VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec
first_frame = cv2.imread(image_files[0])
frame_size = (first_frame.shape[1], first_frame.shape[0])  # (width, height)
video_writer = cv2.VideoWriter(output_video_path, fourcc, frame_rate, frame_size)

# Iterate over image files and write to video
for image_file in image_files:
    frame = cv2.imread(image_file)
    if frame is None:
        print(f"Skipping invalid frame: {image_file}")
        continue
    frame_resized = cv2.resize(frame, frame_size)  # Resize frame if necessary
    video_writer.write(frame_resized)

# Release the VideoWriter
video_writer.release()

print(f"Video saved to {output_video_path}")
