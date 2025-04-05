#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from skimage.feature import hog
import joblib

class KNNDetector:
    def __init__(self):
        rospy.init_node("knn_digit_detector")
        self.bridge = CvBridge()

        model_path = rospy.get_param("~model_path", "knn_digit_model.pkl")
        self.model = joblib.load(model_path)
        rospy.loginfo("KNN model loaded.")

        self.sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("/perception/knn_detect_result", String, queue_size=10)

    def extract_hog(self, img):
        img = cv2.resize(img, (28, 28))
        return hog(img, pixels_per_cell=(4, 4), cells_per_block=(2, 2), feature_vector=True)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            feature = self.extract_hog(gray).reshape(1, -1)
            pred = self.model.predict(feature)[0]

            rospy.loginfo(f"KNN predicted: {pred}")
            self.pub.publish(str(pred))

        except Exception as e:
            rospy.logerr(f"Error in KNN detector: {e}")

if __name__ == "__main__":
    try:
        node = KNNDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
