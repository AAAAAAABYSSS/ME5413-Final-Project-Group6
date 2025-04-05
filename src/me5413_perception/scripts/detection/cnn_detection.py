#!/usr/bin/env python3
import rospy
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class SimpleCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(1, 32, 3, 1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, 1),
            nn.ReLU(),
            nn.MaxPool2d(2)
        )
        self.fc = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64*5*5, 128),
            nn.ReLU(),
            nn.Linear(128, 10)
        )
    def forward(self, x):
        return self.fc(self.conv(x))

class CNNDigitNode:
    def __init__(self):
        rospy.init_node("simple_cnn_detector")
        self.bridge = CvBridge()

        model_path = rospy.get_param("~model_path", "simple_cnn.pt")
        self.model = SimpleCNN()
        self.model.load_state_dict(torch.load(model_path, map_location="cpu"))
        self.model.eval()

        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Grayscale(),
            transforms.Resize((28, 28)),
            transforms.ToTensor()
        ])

        self.sub = rospy.Subscriber("/front/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("/perception/cnn_detect_result", String, queue_size=10)
        rospy.loginfo("Simple CNN Detector Ready")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            tensor = self.transform(gray).unsqueeze(0)  
            with torch.no_grad():
                output = self.model(tensor)
                pred = torch.argmax(output, dim=1).item()
            self.pub.publish(str(pred))
            rospy.loginfo(f"Predicted Digit: {pred}")
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == "__main__":
    try:
        CNNDigitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
