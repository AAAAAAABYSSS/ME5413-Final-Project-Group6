#!/usr/bin/env python
import tf
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class BBoxTransformer:
    def __init__(self):
        rospy.init_node("bbox_transformer", anonymous=True)

        self.current_pose_inv = None  # 存储 map->base_link 的逆变换（即 base_link->map 的反向变换）

        # 订阅 bbox 和 odom
        # rospy.Subscriber("/perception/bbox_markers", MarkerArray, self.bbox_callback)
        # rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        rospy.Subscriber("/perception/marker/nav_goal_marker", Marker, self.arrow_callback)

        # 发布变换后的 bbox
        # self.transformed_pub = rospy.Publisher("/bbox_markers_baselink", MarkerArray, queue_size=1)
        # 发布变换后的 arrow
        self.arrow_pub = rospy.Publisher("/perception/marker/arrow_marker_baselink", Marker, queue_size=1)


        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)
        rospy.loginfo("BBoxTransformer started.")


    def update_pose_from_tf(self):
        """Get transform from map to base and update current_pose"""
        try:
            # Velodyne to map transform
            (trans, rot) = self.tf_listener.lookupTransform(
                "base_link", "map", rospy.Time(0)
            )
            rotation_matrix = R.from_quat(rot).as_matrix()
            translation = np.array(trans)

            T_map_to_base = np.eye(4)
            T_map_to_base[:3, :3] = rotation_matrix
            T_map_to_base[:3, 3] = translation

            self.current_pose_inv = T_map_to_base
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn("TF lookup failed")

    # def odom_callback(self, msg):
    #     """ 获取 map → base_link 的位姿，计算 base_link → map 的逆变换 """
    #     position = msg.pose.pose.position
    #     orientation = msg.pose.pose.orientation

    #     # 构造 map->base_link 的变换矩阵 T_map2base
    #     T_map2base = np.eye(4)
    #     T_map2base[:3, 3] = [position.x, position.y, position.z]
    #     rot_matrix = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
    #     T_map2base[:3, :3] = rot_matrix

    #     self.current_pose_inv = np.linalg.inv(T_map2base)

    # def bbox_callback(self, msg):
    #     """ 将 bbox 从 map 坐标转换到 base_link 坐标 """
    #     if self.current_pose_inv is None:
    #         rospy.logwarn("等待里程计数据...")
    #         return

    #     transformed_array = MarkerArray()

    #     for marker in msg.markers:
    #         # 构建点的齐次坐标
    #         center_point = np.array([
    #             marker.pose.position.x,
    #             marker.pose.position.y,
    #             marker.pose.position.z,
    #             1.0
    #         ])
    #         # 变换到 base_link 坐标
    #         transformed_center = self.current_pose_inv @ center_point

    #         # 朝向变换：marker 原始朝向
    #         q_marker = [
    #             marker.pose.orientation.x,
    #             marker.pose.orientation.y,
    #             marker.pose.orientation.z,
    #             marker.pose.orientation.w
    #         ]
    #         rot_marker = R.from_quat(q_marker)

    #         # 变换后的朝向 = 当前姿态旋转的逆 × 原始 marker 旋转
    #         rot_map2base_mat = self.current_pose_inv[:3, :3]  # 3x3矩阵
    #         rot_map2base = R.from_matrix(rot_map2base_mat)  # 转换为 Rotation 对象
    #         # 旋转相乘
    #         rot_transformed = rot_map2base * rot_marker
    #         q_new = rot_transformed.as_quat()

    #         # 创建新的 Marker
    #         new_marker = Marker()
    #         new_marker.header.frame_id = "base_link"
    #         new_marker.header.stamp = rospy.Time.now()
    #         new_marker.ns = marker.ns
    #         new_marker.id = marker.id
    #         new_marker.type = marker.type
    #         new_marker.action = Marker.ADD

    #         # 设置变换后的位姿
    #         new_marker.pose.position.x = transformed_center[0]
    #         new_marker.pose.position.y = transformed_center[1]
    #         new_marker.pose.position.z = transformed_center[2]

    #         # 设置朝向
    #         new_marker.pose.orientation.x = q_new[0]
    #         new_marker.pose.orientation.y = q_new[1]
    #         new_marker.pose.orientation.z = q_new[2]
    #         new_marker.pose.orientation.w = q_new[3]

    #         # 保持尺寸与颜色
    #         new_marker.scale = marker.scale
    #         new_marker.color = marker.color
    #         new_marker.lifetime = rospy.Duration(0.5)
    #         # new_marker.lifetime = rospy.Duration(0)

    #         transformed_array.markers.append(new_marker)

    #         self.transformed_pub.publish(transformed_array)

    def arrow_callback(self, marker):
        self.update_pose_from_tf()
        if self.current_pose_inv is None:
            rospy.logwarn("等待里程计数据...")
            return

        transformed_marker = Marker()
        transformed_marker.header.frame_id = "base_link"
        transformed_marker.header.stamp = rospy.Time.now()
        transformed_marker.ns = marker.ns
        transformed_marker.id = marker.id
        transformed_marker.type = marker.type
        transformed_marker.action = Marker.ADD

        # 变换 points[0] 和 points[1]
        from geometry_msgs.msg import Point
        transformed_points = []
        for pt in marker.points:
            pt_hom = np.array([pt.x, pt.y, pt.z, 1.0])
            pt_trans = self.current_pose_inv @ pt_hom
            transformed_points.append(Point(x=pt_trans[0], y=pt_trans[1], z=pt_trans[2]))
        transformed_marker.points = transformed_points

        # 保留其它属性
        transformed_marker.scale = marker.scale
        transformed_marker.color = marker.color
        transformed_marker.lifetime = rospy.Duration(1.0)

        self.arrow_pub.publish(transformed_marker)


if __name__ == "__main__":
    try:
        BBoxTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
