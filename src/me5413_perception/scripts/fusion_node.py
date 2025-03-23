import rospy
import json
import numpy as np
import tf
from sensor_msgs.msg import PointCloud2, CameraInfo
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from tf import transformations
import math

class FusionNode:
    def __init__(self):
        rospy.init_node("fusion_node", anonymous=True)

        self.latest_pointcloud = None
        self.camera_intrinsics = None
        self.yolo_targets = []
        self.tf_listener = tf.TransformListener()

        rospy.Subscriber("/front/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/mid/points", PointCloud2, self.pointcloud_callback)
        rospy.Subscriber("/yolo_targets", String, self.yolo_callback)

        self.target_3d_pub = rospy.Publisher("/yolo_targets_3d", String, queue_size=10)
        self.rate = rospy.Rate(30)

    def camera_info_callback(self, msg):
        self.camera_intrinsics = np.array(msg.K).reshape(3, 3)
        rospy.loginfo("Camera intrinsic matrix loaded:\n%s" % self.camera_intrinsics)

    def pointcloud_callback(self, msg):
        try:
            self.latest_pointcloud = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as e:
            rospy.logerr("Failed to convert point cloud: %s" % str(e))

    def yolo_callback(self, msg):
        try:
            self.yolo_targets = json.loads(msg.data)
        except Exception as e:
            rospy.logerr(f"Error parsing YOLO target JSON: {e}")

    def transform_point(self, point, transform):
        tf_point = np.dot(transform, np.array([point[0], point[1], point[2], 1.0]))
        return tf_point[:3]

    def merge_close_boxes(self, matched_boxes, box_width=0.8):
        merged_boxes = []
        used_indices = set()
        box_list = list(matched_boxes.values())

        for i, box1 in enumerate(box_list):
            if i in used_indices:
                continue

            x1, y1, z1, label1, conf1 = box1["X"], box1["Y"], box1["Z"], box1["label"], box1["conf"]
            best_box = box1

            for j, box2 in enumerate(box_list):
                if i == j or j in used_indices:
                    continue

                x2, y2, z2, label2, conf2 = box2["X"], box2["Y"], box2["Z"], box2["label"], box2["conf"]
                
                if abs(x1 - x2) < box_width and abs(z1 - z2) < 1.0:
                    if conf2 > conf1:
                        best_box = box2
                        conf1 = conf2
                    used_indices.add(j)

            merged_boxes.append(best_box)

        rospy.loginfo(f"Merged box count: {len(merged_boxes)} (Original: {len(matched_boxes)})")
        return merged_boxes

    def process_data(self):
        while not rospy.is_shutdown():
            if self.latest_pointcloud is not None and self.camera_intrinsics is not None and self.yolo_targets:
                try:
                    (trans1, rot1) = self.tf_listener.lookupTransform("/base_link", "/velodyne", rospy.Time(0))
                    (trans2, rot2) = self.tf_listener.lookupTransform("/front_camera_optical", "/base_link", rospy.Time(0))
                    transform1 = np.dot(transformations.translation_matrix(trans1), transformations.quaternion_matrix(rot1))
                    transform2 = np.dot(transformations.translation_matrix(trans2), transformations.quaternion_matrix(rot2))
                    transform_matrix = np.dot(transform2, transform1)

                    fx, fy, cx, cy = self.camera_intrinsics[0, 0], self.camera_intrinsics[1, 1], self.camera_intrinsics[0, 2], self.camera_intrinsics[1, 2]

                    matched_boxes = {}

                    for point in self.latest_pointcloud:
                        transformed_point = self.transform_point(point, transform_matrix)
                        X, Y, Z = transformed_point
                        X0, Y0, Z0 = point

                        if Z > 0:
                            u = int((fx * X / Z) + cx)
                            v = int((fy * Y / Z) + cy)

                            for target in self.yolo_targets:
                                u_target, v_target, label, conf = target["u_center"], target["v_center"], target["label"], target["conf"]

                                if abs(u - u_target) < 5 and abs(v - v_target) < 5:
                                    key = (label, conf)
                                    if key not in matched_boxes:
                                        matched_boxes[key] = {
                                            "u_center": u_target,
                                            "v_center": v_target,
                                            "label": label,
                                            "conf": conf,
                                            "X": X0,
                                            "Y": Y0,
                                            "Z": Z0
                                        }
                                    else:
                                        existing_X, existing_Y, existing_Z = matched_boxes[key]["X"], matched_boxes[key]["Y"], matched_boxes[key]["Z"]
                                        distance = math.sqrt((X - existing_X)**2 + (Y - existing_Y)**2 + (Z - existing_Z)**2)

                                        if distance < 1.1:
                                            if conf > matched_boxes[key]["conf"]:
                                                matched_boxes[key].update({"X": X0, "Y": Y0, "Z": Z0, "conf": conf, "label": label})
                                        else:
                                            matched_boxes[(label, conf, len(matched_boxes))] = {"u_center": u_target, "v_center": v_target, "label": label, "conf": conf, "X": X0, "Y": Y0, "Z": Z0}

                    if matched_boxes:
                        merged_boxes = self.merge_close_boxes(matched_boxes, box_width=0.8)
                        self.target_3d_pub.publish(json.dumps(merged_boxes))

                except Exception as e:
                    rospy.logwarn("Failed to transform coordinates: %s" % str(e))

            self.rate.sleep()

if __name__ == "__main__":
    try:
        fusion_node = FusionNode()
        fusion_node.process_data()
    except rospy.ROSInterruptException:
        pass