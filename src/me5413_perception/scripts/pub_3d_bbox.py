#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class BBoxTransformer:
    def __init__(self):
        rospy.init_node("bbox_transformer", anonymous=True)

        self.current_pose_inv = None  # Store the inverse transformation from map to base_link

        # Subscribe to bbox and odom
        rospy.Subscriber("/bbox_markers", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        rospy.Subscriber("/nav_goal_marker", Marker, self.arrow_callback)

        # Publish transformed bbox
        self.transformed_pub = rospy.Publisher("/bbox_markers_baselink", MarkerArray, queue_size=1)
        # Publish transformed arrow
        self.arrow_pub = rospy.Publisher("/arrow_marker_baselink", Marker, queue_size=1)
        self.bbox_list = []

        rospy.loginfo("BBoxTransformer started.")
        rospy.spin()

    def odom_callback(self, msg):
        """ Get the pose from map to base_link and compute the inverse transformation to base_link to map """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Construct the transformation matrix from map to base_link
        T_map2base = np.eye(4)
        T_map2base[:3, 3] = [position.x, position.y, position.z]
        rot_matrix = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
        T_map2base[:3, :3] = rot_matrix

        self.current_pose_inv = np.linalg.inv(T_map2base)

    def bbox_callback(self, msg):
        """ Transform bbox coordinates from map to base_link """
        if self.current_pose_inv is None:
            rospy.logwarn("Waiting for odometry data...")
            return

        transformed_array = MarkerArray()

        for marker in msg.markers:
            # Construct homogeneous coordinates
            center_point = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
                1.0
            ])
            # Transform to base_link coordinates
            transformed_center = self.current_pose_inv @ center_point

            # Transform orientation
            q_marker = [
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w
            ]
            rot_marker = R.from_quat(q_marker)

            # Compute new orientation
            rot_map2base_mat = self.current_pose_inv[:3, :3]  # 3x3 matrix
            rot_map2base = R.from_matrix(rot_map2base_mat)  # Convert to Rotation object
            rot_transformed = rot_map2base * rot_marker
            q_new = rot_transformed.as_quat()

            bbox_data = {
            "id": marker.id,
            "X": transformed_center[0],
            "Y": transformed_center[1],
            "Z": transformed_center[2],
            "qx": q_new[0],
            "qy": q_new[1],
            "qz": q_new[2],
            "qw": q_new[3],
            "width": marker.scale.x,
            "height": marker.scale.y,
            "depth": marker.scale.z,
            "detections": []  
        }
            
        existing_bbox = next((b for b in self.bbox_list if b["id"] == bbox_data["id"]), None)
        if existing_bbox:
            existing_bbox["detections"].extend(bbox_data["detections"]) 
        else:
            self.bbox_list.append(bbox_data)

            # Create new Marker
            new_marker = Marker()
            new_marker.header.frame_id = "base_link"
            new_marker.header.stamp = rospy.Time.now()
            new_marker.ns = marker.ns
            new_marker.id = marker.id
            new_marker.type = marker.type
            new_marker.action = Marker.ADD

            # Set transformed pose
            new_marker.pose.position.x = transformed_center[0]
            new_marker.pose.position.y = transformed_center[1]
            new_marker.pose.position.z = transformed_center[2]

            # Set orientation
            new_marker.pose.orientation.x = q_new[0]
            new_marker.pose.orientation.y = q_new[1]
            new_marker.pose.orientation.z = q_new[2]
            new_marker.pose.orientation.w = q_new[3]

            # Preserve size and color
            new_marker.scale = marker.scale
            new_marker.color = marker.color
            new_marker.lifetime = rospy.Duration(0.5)

            transformed_array.markers.append(new_marker)

        self.transformed_pub.publish(transformed_array)
        rospy.loginfo(f"{len(self.bbox_list)} BBox Data")
    
    def finalize_bbox_labels(self):
        """ 
        Compute the final label for each BBox based on the highest confidence detections 
        accumulated throughout the exploration. 
        """
        rospy.loginfo("Computing final labels for each BBox...")

        for bbox in self.bbox_list:
            if not bbox["detections"]:
                bbox["final_label"] = None
                continue

            # Compute confidence scores for each label
            label_confidence = {}
            for detection in bbox["detections"]:
                label = detection["label"]
                conf = detection["conf"]
                if label in label_confidence:
                    label_confidence[label].append(conf)
                else:
                    label_confidence[label] = [conf]

            # Choose the label with the highest average confidence
            best_label = max(label_confidence, key=lambda l: sum(label_confidence[l]) / len(label_confidence[l]))
            bbox["final_label"] = best_label

        rospy.loginfo("Final labels for all BBoxes have been computed.")

        # **Print final BBox data**
        rospy.loginfo("\n ====== BBox Data ======")
        for bbox in self.bbox_list:
            rospy.loginfo(f"   BBox ID: {bbox['id']}")
            rospy.loginfo(f"   Position: X={bbox['X']:.2f}, Y={bbox['Y']:.2f}, Z={bbox['Z']:.2f}")
            rospy.loginfo(f"   Detected Classes: {[d['label'] for d in bbox['detections']]}")
            rospy.loginfo(f"   Confidence Scores: {[d['conf'] for d in bbox['detections']]}")
            rospy.loginfo(f"   Final Label: {bbox['final_label']}\n")

    def arrow_callback(self, marker):
        if self.current_pose_inv is None:
            rospy.logwarn("Waiting for odometry data...")
            return

        transformed_marker = Marker()
        transformed_marker.header.frame_id = "base_link"
        transformed_marker.header.stamp = rospy.Time.now()
        transformed_marker.ns = marker.ns
        transformed_marker.id = marker.id
        transformed_marker.type = marker.type
        transformed_marker.action = Marker.ADD

        # Transform points[0] and points[1]
        from geometry_msgs.msg import Point
        transformed_points = []
        for pt in marker.points:
            pt_hom = np.array([pt.x, pt.y, pt.z, 1.0])
            pt_trans = self.current_pose_inv @ pt_hom
            transformed_points.append(Point(x=pt_trans[0], y=pt_trans[1], z=pt_trans[2]))
        transformed_marker.points = transformed_points

        # Preserve other properties
        transformed_marker.scale = marker.scale
        transformed_marker.color = marker.color
        transformed_marker.lifetime = rospy.Duration(1.0)

        self.arrow_pub.publish(transformed_marker)

if __name__ == "__main__":
    try:
        BBoxTransformer()
    except rospy.ROSInterruptException:
        pass
