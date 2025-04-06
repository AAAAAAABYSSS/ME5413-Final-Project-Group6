#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from threading import Lock


class BBoxEvaluation:
    def __init__(self):
        rospy.init_node("bbox_evaluation", anonymous=True)

        # Initialize storage variables
        self.ground_truth_boxes = []  # (x, y, w, h, id)
        self.predicted_boxes = []  # (x, y, w, h)
        self.lock = Lock()
        self.dest_boxes_x = rospy.get_param("~dest_boxes_x", 0.0)
        self.dest_boxes_y = rospy.get_param(
            "~dest_boxes_y", [-6.0, -10.0, -14.0, -18.0]
        )
        self.ax_xlim = rospy.get_param("~ax_xlim", [-1.5, 23.5])
        self.ax_ylim = rospy.get_param("~ax_ylim", [-23.5, 1.5])
        self.dest_extent = rospy.get_param("~dest_extent", 0.8)
        self.iou_threshold = rospy.get_param("~iou_threshold", 0.5)
        self.linewidth = rospy.get_param("~linewidth", 0.5)
        self.fontsize = rospy.get_param("~fontsize", 5)

        # Initialize plt and ax
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(self.ax_xlim[0], self.ax_xlim[1])
        self.ax.set_ylim(self.ax_ylim[0], self.ax_ylim[1])
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Box Positions in Gazebo Map Frame")

        # Subscribe to topics
        rospy.Subscriber(
            "/gazebo/ground_truth/box_markers", MarkerArray, self.callback_ground_truth
        )
        rospy.Subscriber(
            "/perception/marker/bbox_markers", MarkerArray, self.callback_predicted
        )

    def callback_ground_truth(self, msg):
        positions = []
        for marker in msg.markers:
            # Convert from World Frame to Map Frame: rot(z,90°)
            x = marker.pose.position.y
            y = marker.pose.position.x * (-1)
            w = marker.scale.y
            h = marker.scale.x
            id = marker.id % 20
            positions.append((x, y, w, h, id))
        for dest_y in self.dest_boxes_y:
            positions.append(
                (self.dest_boxes_x, dest_y, self.dest_extent, self.dest_extent, -1)
            )

        with self.lock:
            self.ground_truth_boxes = positions

    def callback_predicted(self, msg):
        preds = []
        for marker in msg.markers:
            if marker.ns == "box" or marker.ns == "bridge":
                x = marker.pose.position.x
                y = marker.pose.position.y
                w = marker.scale.x
                h = marker.scale.y
                preds.append((x, y, w, h))

        with self.lock:
            self.predicted_boxes = preds

    def update_plot(self):
        boxes_drawn = []

        # Clear the content of the previous round of drawing
        while not rospy.is_shutdown():
            for artist in boxes_drawn:
                try:
                    artist.remove()
                except ValueError:
                    pass
            boxes_drawn.clear()

            with self.lock:
                gt = list(self.ground_truth_boxes)
                pred = list(self.predicted_boxes)

            # Add a legend
            if not hasattr(self, "legend_drawn"):
                from matplotlib.lines import Line2D

                legend_elements = [
                    Rectangle(
                        (0, 0),
                        1,
                        1,
                        edgecolor="blue",
                        facecolor="cyan",
                        alpha=0.6,
                        label="Ground Truth BBox",
                    ),
                    Line2D(
                        [0],
                        [0],
                        marker="x",
                        color="blue",
                        linestyle="None",
                        label="GT Center",
                    ),
                    Rectangle(
                        (0, 0),
                        1,
                        1,
                        edgecolor="red",
                        facecolor="none",
                        label="Predicted BBox",
                    ),
                    Line2D(
                        [0],
                        [0],
                        marker="o",
                        color="red",
                        linestyle="None",
                        label="Pred Center",
                    ),
                ]
                self.ax.legend(handles=legend_elements, loc="upper right", fontsize=7)
                self.legend_drawn = True

            # Draw the ground truth boxes (blue squares + blue x)
            for i, (x, y, w, h, id) in enumerate(gt):
                rect = Rectangle(
                    (x - w / 2, y - h / 2),
                    w,
                    h,
                    linewidth=self.linewidth,
                    edgecolor="blue",
                    facecolor="cyan",
                    alpha=0.6,
                )
                self.ax.add_patch(rect)
                boxes_drawn.append(rect)

                txt_x = self.ax.text(
                    x,
                    y,
                    "x",
                    color="blue",
                    ha="center",
                    va="center",
                    fontsize=self.fontsize,
                )
                # Bridge and destination boxes do not have labels
                if id == (99 % 20) or x == self.dest_boxes_x:
                    txt_id = self.ax.text(
                        x,
                        y + 0.5,
                        f"[{i+1}]",
                        color="black",
                        fontsize=self.fontsize,
                        ha="center",
                    )
                else:
                    txt_id = self.ax.text(
                        x,
                        y + 0.5,
                        f"[{i+1}_{id}]",
                        color="black",
                        fontsize=self.fontsize,
                        ha="center",
                    )
                boxes_drawn.extend([txt_x, txt_id])

            # Draw the predicted boxes (red rectangle + red point)
            for x, y, w, h in pred:
                rect = Rectangle(
                    (x - w / 2, y - h / 2),
                    w,
                    h,
                    linewidth=self.linewidth,
                    edgecolor="red",
                    facecolor="none",
                )
                self.ax.add_patch(rect)
                boxes_drawn.append(rect)

                # Red point
                (dot,) = self.ax.plot(x, y, "ro", markersize=2)
                boxes_drawn.append(dot)

            self.fig.canvas.draw_idle()
            plt.pause(1.0)

            # Matching predicted boxes and GT boxes
            matched_gt_indices = set()
            tp = 0
            fp = 0

            for pred_box in pred:
                best_iou = 0
                best_gt_idx = -1
                for idx, gt_box in enumerate(gt):
                    if idx in matched_gt_indices:
                        continue  # Already matched, skipping
                    iou = self.compute_iou(pred_box, gt_box)
                    if iou > best_iou:
                        best_iou = iou
                        best_gt_idx = idx

                if best_iou >= self.iou_threshold:
                    tp += 1
                    matched_gt_indices.add(best_gt_idx)
                else:
                    fp += 1

            # Calculate IoU list for visualization
            iou_list = []
            for gt_box in gt:
                best_iou = 0
                for pred_box in pred:
                    iou = self.compute_iou(pred_box, gt_box)
                    if iou > best_iou:
                        best_iou = iou
                iou_list.append(best_iou)

            # Unmatched GT is counted as FN
            fn = len(gt) - len([iou for iou in iou_list if iou > 0])

            # Visualize table
            header_str = " | ".join([f"[{i+1:>3}]" for i in range(len(iou_list))])
            iou_str = " | ".join([f"{iou:.3f}" for iou in iou_list])

            print("\n" + "=" * 60)
            print(f"IoUs for {len(gt)} ground truth boxes:")
            print(header_str)
            print(iou_str)
            print("=" * 60)

            # Output matching statistics
            precision = tp / (tp + fp) if (tp + fp) > 0 else 0
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0
            f1_score = (
                (2 * precision * recall) / (precision + recall)
                if (precision + recall) > 0
                else 0
            )

            print(f"TP: {tp}, FP: {fp}, FN: {fn}")
            print(
                f"Estimated @{self.iou_threshold}, Precision: {precision:.3f}, Recall: {recall:.3f}, F1 Score: {f1_score:.3f}"
            )
            print("=" * 60)

    def compute_iou(self, box1, box2):
        # box1 and box2: (x, y, w, h, id)
        if box2[4] == (99 % 20):
            x1_min = box1[0] - box1[2] / 2
            x1_max = box1[0] + box1[2] / 2
            y1_min = box1[1] - box1[3] / 2
            y1_max = box1[1] + box1[3] / 2

            x2_min = box2[0] - box2[2] / 2
            x2_max = box2[0] + box2[2] / 2
            y2_min = box2[1] - box2[3] / 2
            y2_max = box2[1] + box2[3] / 2
        else:
            x1_min = box1[0] - box1[2] / 2
            x1_max = box1[0] + box1[2] / 2
            y1_min = box1[1] - box1[3] / 2
            y1_max = box1[1] + box1[3] / 2

            x2_min = box2[0] - box2[2] / 2
            x2_max = box2[0] + box2[2] / 2
            y2_min = box2[1] - box2[3] / 2
            y2_max = box2[1] + box2[3] / 2

        inter_x_min = max(x1_min, x2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_min = max(y1_min, y2_min)
        inter_y_max = min(y1_max, y2_max)

        inter_area = max(0, inter_x_max - inter_x_min) * max(
            0, inter_y_max - inter_y_min
        )
        area1 = (x1_max - x1_min) * (y1_max - y1_min)
        area2 = (x2_max - x2_min) * (y2_max - y2_min)
        union_area = area1 + area2 - inter_area

        iou = inter_area / union_area if union_area > 0 else 0
        return iou

    def run(self):
        """Run ROS listener"""
        self.update_plot()
        rospy.spin()


if __name__ == "__main__":
    processor = BBoxEvaluation()
    processor.run()
