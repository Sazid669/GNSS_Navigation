
import rospy
from ultralytics import YOLO
import urllib.request
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

model = YOLO('yolov8m.pt')
bridge = CvBridge()
output_dir = os.path.join(os.getcwd(), 'yolo_output')
os.makedirs(output_dir, exist_ok=True)
frame_count = 0
video_output_path = os.path.join(output_dir, 'output_video.avi')
frame_rate = 30  # FPS
frame_size = (1280, 720)
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec
video_writer = cv2.VideoWriter(video_output_path, fourcc, frame_rate, frame_size)

def image_callback(msg):
    global frame_count
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    frame_resized = cv2.resize(frame, frame_size)
    results = model(frame_resized)

    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = box.conf[0]
        label = box.cls[0]
        cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame_resized, f'{model.names[int(label)]}: {conf:.2f}', (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
    # Save the frame with bounding boxes
    output_path = os.path.join(output_dir, f"frame_{frame_count}.jpg")
    saved = cv2.imwrite(output_path, frame_resized)
    if saved:
        rospy.loginfo(f"Frame {frame_count} saved to {output_path}")

    video_writer.write(frame_resized)

    frame_count += 1

    try:
        ros_image = bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
        image_pub.publish(ros_image)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    rospy.init_node('yolov8_node')
    image_pub = rospy.Publisher('/yolov8/detections', Image, queue_size=10)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()
    video_writer.release()
    cv2.destroyAllWindows()