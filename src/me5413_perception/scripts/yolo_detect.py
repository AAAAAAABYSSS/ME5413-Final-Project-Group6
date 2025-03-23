#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import torch
import json
import os
import rospkg  # Get ROS package location
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class YOLODetector:
    def __init__(self):
        rospy.init_node("yolo_detector", anonymous=True)

        self.bridge = CvBridge()
        self.yolo_pub = rospy.Publisher("/yolo_targets", String, queue_size=10)
        self.result_pub = rospy.Publisher("/front/image_result", Image, queue_size=1)

        # Get the correct model path
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("me5413_perception")
        model_path = os.path.join(package_path, "models", "weights1.pt")  # Ensure model is in models directory

        rospy.loginfo(f"Loading YOLO model from {model_path}")

        # Check if model file exists
        if not os.path.exists(model_path):
            rospy.logerr(f"Model file not found: {model_path}")
            rospy.signal_shutdown("Model file missing.")
            return

        try:
            self.model = YOLO(model_path)  # Load YOLO model
            rospy.loginfo("YOLO model loaded successfully.")

            # Ensure model is loaded before subscribing to images
            self.image_sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback)

        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            rospy.signal_shutdown("Model loading failed.")
            return

    def image_callback(self, msg):
        if not hasattr(self, "model"):
            rospy.logwarn("YOLO model not loaded yet, skipping image processing.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert ROS image
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB

            results = self.model(frame, conf=0.1)  # YOLO object detection

            if not results or not results[0].boxes:
                rospy.logwarn("No targets detected!")
                return

            target_data = []

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                    label = int(box.cls)  # Object class (0-9)
                    conf = float(box.conf)  # Confidence score

                    u_center = (x1 + x2) // 2
                    v_center = (y1 + y2) // 2

                    target_dict = {
                        "u_center": u_center,
                        "v_center": v_center,
                        "label": label,
                        "conf": conf
                    }
                    target_data.append(target_dict)

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Add label
                    text = f"{label} ({conf:.2f})"
                    cv2.putText(frame, text, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Publish target data
            self.yolo_pub.publish(json.dumps(target_data))

            # Show image only if a GUI environment is available
            if "DISPLAY" in os.environ:
                cv2.imshow("YOLO Detection", frame)
                cv2.waitKey(1)

            # Publish processed image
            result_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            self.result_pub.publish(result_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    try:
        node = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
