#!/usr/bin/env python3
import tf
import json
import rospy
import actionlib
import numpy as np
from scipy.ndimage import label
from transitions import Machine
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import PoseStamped, Twist
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import MarkerArray, Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RobotStateMachine(Machine):
    def __init__(self, robot):

        # Initialize the state machine
        self.robot = robot

        # Define the states
        states = [
            'idle',                             # Initial state before navigation begins
            'waiting_for_bbox',                 # Waiting for the first batch of bounding box data
            
            # Pre-bridge box navigation
            'navigating_to_furthest_box',       # Navigating to the furthest unmatched box
            'evaluating_bridge_conditions',     # Evaluating whether bridge conditions are satisfied

            # Bridge crossing sequence
            'bridge_goal_sent',                 # Bridge goal has been published
            'waiting_at_bridge',                # Waiting at the bridge before crossing
            'crossing_bridge',                  # Performing bridge crossing (e.g., reverse + open bridge)   
            
            
            # Post-bridge navigation
            'navigating_to_post_bridge_goal',   # Moving to a fixed post-bridge position
            'post_bridge_direct_box_found',     # Target box found directly after crossing
            'post_bridge_scanning',             # Performing rotation scan to find box
            'post_bridge_rarest_box_fallback',  # No box found after scan, fallback to rarest
           
            # Fallback
            'fallback_navigation',              # No boxes available, exploring fallback region
        ]

        # Define the state transitions
        transitions = [
            # Startup
            {'trigger': 'start', 'source': 'idle', 'dest': 'waiting_for_bbox'},

            # Got box before bridge
            {'trigger': 'received_bbox', 'source': 'waiting_for_bbox', 'dest': 'navigating_to_furthest_box'},
            {'trigger': 'no_box_available', 'source': 'waiting_for_bbox', 'dest': 'fallback_navigation'},

            # After box nav: evaluate bridge
            {'trigger': 'check_bridge_ready', 'source': 'navigating_to_furthest_box', 'dest': 'evaluating_bridge_conditions'},

            # Bridge logic
            {'trigger': 'send_bridge_goal', 'source': 'evaluating_bridge_conditions', 'dest': 'bridge_goal_sent'},
            {'trigger': 'arrived_at_bridge', 'source': 'bridge_goal_sent', 'dest': 'waiting_at_bridge'},
            {'trigger': 'start_crossing', 'source': 'waiting_at_bridge', 'dest': 'crossing_bridge'},
            {'trigger': 'reached_post_bridge', 'source': 'crossing_bridge', 'dest': 'navigating_to_post_bridge_goal'},

            # Post-bridge logic
            {'trigger': 'post_bridge_target_found', 'source': 'navigating_to_post_bridge_goal', 'dest': 'post_bridge_direct_box_found'},
            {'trigger': 'start_scanning', 'source': 'navigating_to_post_bridge_goal', 'dest': 'post_bridge_scanning'},
            {'trigger': 'fallback_to_rarest', 'source': 'post_bridge_scanning', 'dest': 'post_bridge_rarest_box_fallback'},

            # Fallback recovery
            {'trigger': 'fallback_available', 'source': '*', 'dest': 'fallback_navigation'},
            {'trigger': 'new_box_detected', 'source': 'fallback_navigation', 'dest': 'navigating_to_furthest_box'},

            # Reset or full restart
            {'trigger': 'reset', 'source': '*', 'dest': 'idle'}
        ]


        # Use transitions Machine to create FSM

        self.states = states
        self.transitions = transitions

        super().__init__(model=self, states=states, transitions=transitions, initial='idle', auto_transitions=False)
        
         # Optional: Print state on every entry
        for state in self.states:
            method_name = f'on_enter_{state}'
            if not hasattr(self, method_name):
                setattr(self, method_name, self.default_on_enter)
       
    def default_on_enter(self):
        print(f"[StateMachine] Entered state: {self.state}")

    def on_enter_waiting_for_bbox(self):
        rospy.loginfo("[FSM] Entered: waiting_for_bbox")
        
        if self.robot.last_bbox_msg and any(m.ns == "box" for m in self.robot.last_bbox_msg.markers):
            rospy.loginfo("[FSM] Already have bounding boxes. Transitioning to navigation.")
            self.received_bbox()
        else:
            rospy.loginfo("[FSM] No bbox available yet. Waiting for data.")

    def on_enter_navigating_to_furthest_box(self):
        rospy.loginfo("[FSM] Entered: navigating_to_furthest_box")
        self.robot.exit_fallback_mode()
        if self.robot.last_bbox_msg:
            self.robot.find_and_publish_new_goal(self.robot.last_bbox_msg)
            self.robot.begin_nav = True 

    def on_enter_bridge_goal_sent(self):
        rospy.loginfo("[FSM] Entered: bridge_goal_sent")
        self.robot.last_goal_position = self.robot.bridge_position
        # self.robot.publish_goal(self.robot.bridge_position, yaw_deg=180.0)
        self.robot.publish_bridge_goal_with_action(self.robot.bridge_position, yaw_deg=180.0)

        self.robot.publish_arrow_marker(self.robot.bridge_position)

    def on_enter_waiting_at_bridge(self):
        rospy.loginfo("[FSM] Entered: waiting_at_bridge")
        # Maybe wait for /one_rot/end_status

    def on_enter_evaluating_bridge_conditions(self):
        rospy.loginfo("[FSM] Entered: evaluating_bridge_conditions")
        self.send_bridge_goal()

    def on_enter_crossing_bridge(self):
        rospy.loginfo("[FSM] Entered: crossing_bridge")
        self.robot.use_map_localization = False  
        self.robot.cancel_navigation_goal()
        self.robot.open_bridge_pub.publish(Bool(data=True))
        self.robot.move_backwards(duration_sec=6.2, speed=8.2)
        self.robot.publish_arrow_marker(self.robot.post_bridge_position)

    def on_enter_navigating_to_post_bridge_goal(self):
        rospy.loginfo("[FSM] Entered: navigating_to_post_bridge_goal")
        self.robot.use_map_localization = True  
        self.robot.publish_goal(self.robot.post_bridge_position)
        self.robot.publish_arrow_marker(self.robot.post_bridge_position)

    def on_enter_post_bridge_direct_box_found(self):
        rospy.loginfo("[FSM] Entered: post_bridge_direct_box_found")
        self.robot.publish_goal(self.robot.target_box_position)

    def on_enter_post_bridge_scanning(self):
        rospy.loginfo("[FSM] Entered: post_bridge_scanning")
        self.robot.perform_rotation_scan()

    def on_enter_post_bridge_rarest_box_fallback(self):
        rospy.loginfo("[FSM] Entered: post_bridge_rarest_box_fallback")
        rarest_box = self.robot.target_box_found()
        if rarest_box:
            self.robot.publish_goal(rarest_box["position"])
            self.robot.publish_arrow_marker(rarest_box["position"])

    def on_enter_fallback_navigation(self):
        rospy.loginfo("[FSM] Entered: fallback_navigation")
        self.robot.enter_fallback_mode()
        goal = self.robot.find_fallback_goal()
        if goal:
            self.robot.publish_goal(goal)
            self.robot.publish_arrow_marker(goal)

    def received_bbox(self):
        self.trigger("received_bbox")

    def no_box_available(self):
        self.trigger("no_box_available")

    def send_bridge_goal(self):
        self.trigger("send_bridge_goal")

    def new_box_detected(self):
        self.trigger("new_box_detected")

    def arrived_at_bridge(self):
        self.trigger("arrived_at_bridge")

    def start_crossing(self):
        self.trigger("start_crossing")

    def reached_post_bridge(self):
        self.trigger("reached_post_bridge")

    def post_bridge_target_found(self):
        self.trigger("post_bridge_target_found")

    def start_scanning(self):
        self.trigger("start_scanning")

    def fallback_to_rarest(self):
        self.trigger("fallback_to_rarest")

    def reset(self):
        self.trigger("reset")




class FurthestBoxNavigator:
    def __init__(self):
        rospy.init_node("furthest_box_navigator", anonymous=True)

        # 0. Params
        self.overlap_threshold = rospy.get_param("~overlap_threshold", 0.6)
        self.bridge_length = rospy.get_param("~bridge_length", 4.0)
        self.box_size = rospy.get_param("~box_big_size", 0.8)
        self.yaw_bin_size = rospy.get_param("~yaw_bin_size", 30)
        self.fallback_grid_resolution = rospy.get_param("~fallback_grid_resolution", 0.2)
        self.x_range = rospy.get_param("~x_range", [11.0, 19.0])
        self.y_range = rospy.get_param("~y_range", [-22.0, -2.0])
        self.bridge_pos = rospy.get_param("~bridge_pos", 9.0)

        ## State
        self.tf_listener = tf.TransformListener()
        self.current_pose = np.eye(4)
        self.begin_nav= False

        ## Bridge
        self.bridge_ready = False
        self.bridge_goal_sent = False
        self.bridge_found = False
        self.bridge_position = None
        self.post_bridge_position = None
        self.use_map_localization = True

        ## Box
        self.target_box_position = None
        self.last_goal_position = None
        self.last_bbox_msg = None
        self.current_box = MarkerArray()
        self.rarest_label = None
        self.unmatched_box_count = 0

        self.fusion_info = {}
        self.has_received_bbox = False

        self.in_fallback_mode = False
        self.publish_next_goal = False
        
        self.has_crossed_bridge = False
        self.label_set = set()

        self.crossed = False
        self.postbridge_step = 0

        # 1. State machine FIRST!
        self.state_machine = RobotStateMachine(self)
        self.state_machine.start()

        # 2. THEN register callbacks
        rospy.Subscriber("/perception/marker/bbox_markers_fusion", MarkerArray, self.bbox_callback)
        rospy.Subscriber("/perception/fusion_box_labels", String, self.fusion_info_callback)
        rospy.Subscriber("/one_rot/end_status", Bool, self.status_callback, queue_size=10)

        # 3. THEN timers and clients
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/perception/marker/nav_goal_marker", Marker, queue_size=1)
        self.open_bridge_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=1)

        # 4. Timers
        rospy.Timer(rospy.Duration(1.0), self.update_pose_from_tf)
        # rospy.Timer(rospy.Duration(1.0), self.periodic_bridge_check)
        rospy.Timer(rospy.Duration(1.0), self.begin_nav_publish_goal)
        rospy.Timer(rospy.Duration(1.0), self.try_bridge_ready)
        
        # 5. Action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # self.use_map_localization = True
        # self.post_bridge_position = [4.5, -10.0, 0.0]  
        # self.state_machine.to_navigating_to_post_bridge_goal()

        rospy.loginfo("[MoveBaseClient] Connected to move_base action server.")
        rospy.loginfo("FurthestBoxNavigator Initialization completed")
        rospy.spin()

    def status_callback(self, msg):

        self.publish_next_goal = msg.data
        rospy.loginfo(f"[Status] Goal reached in state: {self.state_machine.state}, {self.publish_next_goal}")

        if not self.begin_nav:
            return
        
        # if self.state_machine.state == "bridge_goal_sent":
        #     rospy.loginfo("[Status] Bridge goal sent.")
        #     self.state_machine.arrived_at_bridge()

        if not self.publish_next_goal:
            return
        
                # Handle while in navigating_to_furthest_box
        elif self.state_machine.state == "navigating_to_furthest_box":
            if self.unmatched_box_count > 0:
                rospy.loginfo("[Status] Continuing navigation with new bbox.")
                self.find_and_publish_new_goal(self.last_bbox)

            if self.bridge_ready:
                rospy.loginfo("[Status] Bridge is ready. Switching to bridge evaluation.")
                self.state_machine.check_bridge_ready()
            return
        
        elif self.state_machine.state == "waiting_at_bridge":
            rospy.loginfo("[Status] Waiting at bridge.")
            self.state_machine.start_crossing()

        elif self.state_machine.state == "crossing_bridge":
            rospy.loginfo("[Status] Crossing bridge.")
            self.state_machine.reached_post_bridge()

        elif self.state_machine.state == "navigating_to_post_bridge_goal":
            if self.target_box_found():
                rospy.loginfo("[Status] Target box found after bridge.")
                self.state_machine.post_bridge_target_found()
            else:
                rospy.loginfo("[Status] No target box found after bridge.")
                self.state_machine.start_scanning()

        elif self.state_machine.state == "post_bridge_scanning":
            if self.target_box_found():
                rospy.loginfo("[Status] Target box found after scanning.")
                self.state_machine.post_bridge_target_found()
            else:
                rospy.loginfo("[Status] No target box found after scanning.")
                self.state_machine.fallback_to_rarest()

        elif self.state_machine.state == "post_bridge_rarest_box_fallback":
            if self.target_box_found():
                rospy.loginfo("[Status] Target box found after fallback. Transitioning to direct box navigation.")
                self.state_machine.post_bridge_target_found()
            else:
                rospy.loginfo("[Status] Still no target box found after fallback.")

        elif self.state_machine.state == "fallback_navigation":
            if self.last_bbox_msg and any(m.ns == "box" for m in self.last_bbox_msg.markers):
                self.state_machine.new_box_detected()
            else:
                rospy.loginfo("[Status] Still no box.")

    def on_bridge_goal_done(self, status, result):
        if self.state_machine.state == "bridge_goal_sent":
            self.state_machine.arrived_at_bridge()

    def begin_nav_publish_goal(self, event=None):
        if self.begin_nav:
            rospy.loginfo("[Navigator] Navigation already started.")
            return 
        if not self.begin_nav:
            fallback_goal = self.find_fallback_goal()
            if fallback_goal:
                rospy.loginfo("[Navigator-begin] Starting navigation in fallback mode.")
                self.enter_fallback_mode()
                self.last_goal_position = fallback_goal
                self.publish_goal(fallback_goal)
                self.publish_arrow_marker(fallback_goal)
                self.begin_nav = True
            return

    def bbox_callback(self, msg):
        self.has_received_bbox = True
        self.current_box = msg
        self.last_bbox = MarkerArray()
        unmatched_box_count = 0
        label_set = set()
        label_count = dict()

        for marker in msg.markers:
            marker_id = str(marker.id)
            if marker.ns != "box":
                if marker.ns == "bridge":
                    self.bridge_found = True
                    rospy.loginfo("[BBoxCallback] Bridge marker found.")
                continue

            is_unmatched = True

            if marker_id in self.fusion_info:
                info = self.fusion_info[marker_id]
                if marker.pose.position.x > 5.0:
                    is_unmatched = not info.get("matched", False)
                    if info.get("matched", True):
                        for h in info.get("history", []):
                            label = h["label"]
                            label_set.add(label)
                            label_count[label] = label_count.get(label, 0) + 1
                    else:
                        self.last_bbox.markers.append(marker)

                    if is_unmatched:
                        self.last_bbox.markers.append(marker)
                        unmatched_box_count += 1
            else:
                self.last_bbox.markers.append(marker)

        self.label_set = label_set
        self.last_bbox_msg = self.last_bbox
        rospy.loginfo(f"[BBoxCallback] Unmatched boxes received: {unmatched_box_count}")

        if label_count:
            self.rarest_label = min(label_count.items(), key=lambda x: x[1])[0]
            rospy.loginfo(f"[BBoxCallback] Rarest box label: {self.rarest_label}")
        else:
            rospy.loginfo("[BBoxCallback] No labels found to determine rarest.")

        # start state machine transitions
        self.unmatched_box_count = unmatched_box_count
        if self.state_machine.state == "waiting_for_bbox":
            if unmatched_box_count > 0:
                self.state_machine.received_bbox()
            else:
                self.state_machine.no_box_available()
            return

        if self.in_fallback_mode and unmatched_box_count > 0:
            rospy.loginfo("[BBoxCallback] New boxes during fallback. Triggering transition.")
            self.state_machine.new_box_detected()
            return

        if self.bridge_ready and self.in_fallback_mode and self.begin_nav:
            rospy.loginfo("[BBoxCallback] Bridge ready during fallback. Switching to bridge goal.")
            self.exit_fallback_mode()
            self.last_goal_position = self.bridge_position
            self.publish_goal(self.bridge_position, yaw_deg=180.0)
            self.publish_arrow_marker(self.bridge_position)
            return
        
    def try_bridge_ready(self, event=None):
        box_count = sum(1 for m in self.current_box.markers if m.ns == "box" and m.pose.position.x > self.bridge_pos)
        bridge_marker = next((m for m in self.current_box.markers if m.ns == "bridge"), None)
        any_unmatched = any(
            not self.fusion_info.get(str(m.id), {}).get("matched", True)
            for m in self.current_box.markers
            if m.ns == "box" and m.pose.position.x > 5.0
        )
        rospy.loginfo(f"[BridgeCheck] Box count: {box_count}, Bridge marker found: {bridge_marker is not None}, Unmatched boxes: {any_unmatched}")
        if not any_unmatched and box_count >= 10 and len(self.label_set) >= 4 and bridge_marker:
            rospy.loginfo("[BridgeCheck0] Bridge ready conditions met.")
            self.bridge_ready = True
            self.bridge_position = [9.8,
                                    bridge_marker.pose.position.y, 0.0]
            self.post_bridge_position = [4.5,
                                            self.bridge_position[1], 0.0]
            
            rospy.loginfo(f"[BridgeCheck] Bridge position: {self.bridge_position}, Post-bridge position: {self.post_bridge_position}")
            rospy.loginfo(f"[BridgeCheck] box_count={box_count}, label_count={len(self.label_set)}, bridge_marker={bridge_marker is not None}, any_unmatched={any_unmatched}")

    def target_box_found(self):
        for marker_id, info in self.fusion_info.items():
            if info.get("matched"):
                for h in info.get("history", []):
                    if h.get("label") == self.rarest_label:
                        for marker in self.last_bbox_msg.markers:
                            if str(marker.id) == marker_id and marker.pose.position.x < 4.0:
                                position = [
                                    marker.pose.position.x,
                                    marker.pose.position.y,
                                    marker.pose.position.z
                                ]
                                self.target_box_position = position
                                rospy.loginfo(f"[TargetBox] Found target box: {marker_id} at {position}")
                                return {"id": marker_id, "position": position}
        return None

    def publish_bridge_goal_with_action(self, position, yaw_deg=180.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]

        q = R.from_euler('z', np.deg2rad(yaw_deg)).as_quat()
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base_client.send_goal(goal, done_cb=self.on_bridge_goal_done)
        rospy.loginfo(f"[BridgeGoal] Sent bridge goal via actionlib: {position}, yaw={yaw_deg:.1f}")

    def find_and_publish_new_goal(self, msg):
        
        max_distance = -1
        furthest_box_position = None
        trans, rot = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        T = np.eye(4)
        T[:3, :3] = R.from_quat(rot).as_matrix()
        T[:3, 3] = trans
        self.current_pose = T
        
        for marker in msg.markers:
            if marker.ns != "box":
                continue
            if marker.pose.position.x <= self.bridge_pos:
                continue  
            
            box_pos = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])
            box_pos_target = box_pos.copy()

            box_extent = np.array(
                [
                    marker.scale.x,
                    marker.scale.y,
                    marker.scale.z,
                ]
            )

            # Plan A
            # if box_extent[1] < 0.8:
            #     box_pos_target[1] = box_pos[1] - box_extent[1] / 2 + 0.8 + 0.6
            #     if box_pos_target[1] > -2.0:
            #         box_pos_target[1] = box_pos[1] + box_extent[1] / 2 - 0.6

            # Plan B
            if box_pos[1] > -4.0:
                box_pos_target[1] = box_pos[1] - 0.8
            elif box_pos[1] < -4.0:
                box_pos_target[1] = box_pos[1] + 0.8
                
            robot_pos = self.current_pose[:3, 3]
            distance = np.linalg.norm(box_pos_target - robot_pos)
            rospy.loginfo(f"[Navigator] Box position: {box_pos_target}, Robot position: {robot_pos}, Distance: {distance}")

            if distance > max_distance:
                max_distance = distance
                furthest_box_position = box_pos_target

        if furthest_box_position is not None and self.begin_nav:
            self.last_goal_position = furthest_box_position
            rospy.loginfo(f"[Navigator-pubgoal] Furthest box position: {robot_pos} to {furthest_box_position}")
            self.publish_goal(furthest_box_position)
            self.publish_arrow_marker(furthest_box_position)
            return furthest_box_position

    def find_fallback_goal(self):
        x_vals = np.arange(self.x_range[0], self.x_range[1], self.fallback_grid_resolution)
        y_vals = np.arange(self.y_range[0], self.y_range[1], self.fallback_grid_resolution)
        X, Y = np.meshgrid(x_vals, y_vals)
        mask = np.ones_like(X, dtype=bool)

        center_x, center_y = 14.0, -12.0  
        center_box_size = 2.0  

        center_occupied = False

        for marker in self.current_box.markers:
            if marker.ns != "box":
                continue
            bx, by = marker.pose.position.x, marker.pose.position.y

            mask &= ~((X >= bx - self.box_size/2) & (X <= bx + self.box_size/2) &
                    (Y >= by - self.box_size/2) & (Y <= by + self.box_size/2))

            if (abs(bx - center_x) < center_box_size / 2.0) and (abs(by - center_y) < center_box_size / 2.0):
                center_occupied = True

        if center_occupied:
            fallback_goal = [18.0, -3.0, 0.0]  
            rospy.logwarn("[Fallback] Center region occupied. Using fallback goal at left side.")
        else:
            fallback_goal = [center_x, center_y, 0.0]
            rospy.loginfo(f"[Fallback] Center region free. Using central fallback goal: {fallback_goal}")

        return [18.0, -3.0, 0.0]

    def is_near(self, target_position, threshold=0.5):
        current_pos = self.current_pose[:3, 3]
        dist = np.linalg.norm(np.array(target_position[:2]) - np.array(current_pos[:2]))
        rospy.loginfo(f"[Navigator] Current position: {current_pos}, Target position: {target_position}, Distance: {dist}")
        return dist < threshold
    
    def fusion_info_callback(self, msg):
        try:
            self.fusion_info = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"[FusionInfo] Failed to parse: {e}")

    def update_pose_from_tf(self, event=None):
        try:
            if self.use_map_localization:
                trans, rot = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            else:
                trans, rot = self.tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))

            T = np.eye(4)
            T[:3, :3] = R.from_quat(rot).as_matrix()
            T[:3, 3] = trans
            self.current_pose = T
        except Exception:
            rospy.logwarn("[TF] Failed to update pose")
            
    def perform_rotation_scan(self):
        rospy.loginfo("[Scan] Moving to predefined scan positions for rotation scan...")

        scan_positions = [[4, -8, 0], [4, -16, 0]]
        for idx, position in enumerate(scan_positions):
            rospy.loginfo(f"[Scan] Navigating to scan position {idx+1}: {position}")
            self.publish_goal(position, yaw_deg=180.0)
            rospy.sleep(3.0)
            if self.publish_next_goal:
                continue
            
            result = self.target_box_found()
            if result:
            # if self.target_box_found():
                rospy.loginfo("[Scan] Target found before starting rotation at this point.")
                return

        rospy.logwarn("[Scan] No target found after full rotation scan at both positions.")

    def move_backwards(self, duration_sec=5.0, speed=2.3):
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        twist = Twist()
        twist.linear.x = speed

        while rospy.Time.now() - start_time < rospy.Duration(duration_sec):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())

    def publish_goal(self, position, yaw_deg=0.0):

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]

        q = R.from_euler('z', np.deg2rad(yaw_deg)).as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.goal_pub.publish(msg)
        rospy.loginfo(f"[Goal] Published goal: {position} yaw={yaw_deg:.1f}")

    def publish_arrow_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        start = self.current_pose[:3, 3]
        marker.points = [Point(x=start[0], y=start[1], z=start[2]),
                         Point(x=position[0], y=position[1], z=position[2])]
        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def exit_fallback_mode(self):
        if self.in_fallback_mode:
            rospy.loginfo("[Navigator] Exiting fallback mode.")
        self.in_fallback_mode = False

    def enter_fallback_mode(self):
        if not self.in_fallback_mode:
            rospy.logwarn("[Navigator] Entering fallback mode.")
        self.in_fallback_mode = True

    def cancel_navigation_goal(self):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo("Canceling current navigation goal for rotation.")
        client.cancel_all_goals()

if __name__ == "__main__":
    try:
        FurthestBoxNavigator()
    except rospy.ROSInterruptException:
        pass