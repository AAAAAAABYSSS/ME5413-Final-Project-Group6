#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from threading import Lock
import numpy as np

plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(0, 25)
ax.set_ylim(-3, 22)
ax.set_aspect('equal')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Box Positions in Gazebo")

lock = Lock()
ground_truth_boxes = []   # (x, y)
predicted_boxes = []      # (x, y, w, h)
boxes_drawn = []          # List of drawn rectangles and markers

# Fixed destination boxes for ground truth plus extra points
dest_boxes = [6.0, 10.0, 14.0, 18.0]

def callback_ground_truth(msg):
    global ground_truth_boxes
    positions = []
    for marker in msg.markers:
        x = marker.pose.position.x
        y = marker.pose.position.y
        w = marker.scale.x
        h = marker.scale.y
        positions.append((x, y, w, h))
    for dest_x in dest_boxes:
        positions.append((dest_x, 0.0, 0.8, 0.8))
    with lock:
        ground_truth_boxes = positions

def callback_predicted(msg):
    global predicted_boxes
    preds = []
    for marker in msg.markers:
        if marker.ns == "box":
            x = marker.pose.position.x
            y = marker.pose.position.y
            w = marker.scale.x
            h = marker.scale.y
            preds.append((x, y, w, h))
    with lock:
        predicted_boxes = preds

def update_plot():
    global boxes_drawn
    while not rospy.is_shutdown():
        with lock:
            gt = list(ground_truth_boxes)
            pred = list(predicted_boxes)

        # Clear old images
        for artist in boxes_drawn:
            artist.remove()
        boxes_drawn = []

        # Draw the ground truth boxes (blue squares + blue x)
        for i, (x, y, w, h) in enumerate(gt):
            rect = Rectangle((x - w / 2, y - h / 2), w, h,
                             linewidth=0.5, edgecolor='blue', facecolor='cyan', alpha=0.6)
            ax.add_patch(rect)
            boxes_drawn.append(rect)

            txt_x = ax.text(x, y, 'x', color='blue', ha='center', va='center', fontsize=10, fontweight='bold')
            txt_id = ax.text(x, y + 0.5, f"[{i+1}]", color='black', fontsize=8, ha='center')
            boxes_drawn.extend([txt_x, txt_id])

        # Draw the predicted boxes (red rectangle + red point)
        for (x, y, w, h) in pred:
            rect = Rectangle((x - w / 2, y - h / 2), w, h,
                             linewidth=0.5, edgecolor='red', facecolor='none')
            ax.add_patch(rect)
            boxes_drawn.append(rect)

            # red point
            dot, = ax.plot(x, y, 'ro')
            boxes_drawn.append(dot)

        fig.canvas.draw_idle()
        plt.pause(0.1)

        # 评估 IoU 匹配（可打印或后续用于 TP/FP/FN）
        matched = set()
        for pred_box in pred:
            best_iou = 0
            best_gt = None
            for gt_box in gt:
                iou = compute_iou(pred_box, (gt_box[0], gt_box[1], 0.8, 0.8))
                if iou > best_iou:
                    best_iou = iou
                    best_gt = gt_box

        # Calculate the IoU list of GT and prediction results for each frame (fixed length)
        iou_list = []
        for gt_box in gt:
            gt_rect = (gt_box[0], gt_box[1], 0.8, 0.8)
            best_iou = 0
            for pred_box in pred:
                iou = compute_iou(pred_box, gt_rect)
                if iou > best_iou:
                    best_iou = iou
            iou_list.append(best_iou)

        # Output IoU list, numbered
        header_str = " | ".join([f"[{i+1:>3}]" for i in range(len(iou_list))])
        iou_str = " | ".join([f"{iou:.3f}" for iou in iou_list])

        print("\n" + "="*60)
        print(f"IoUs for {len(gt)} ground truth boxes:")
        print(header_str)
        print(iou_str)
        print("="*60)



def compute_iou(box1, box2):
    # box1, box2: (x, y, w, h)
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

    inter_area = max(0, inter_x_max - inter_x_min) * max(0, inter_y_max - inter_y_min)
    area1 = (x1_max - x1_min) * (y1_max - y1_min)
    area2 = (x2_max - x2_min) * (y2_max - y2_min)
    union_area = area1 + area2 - inter_area

    iou = inter_area / union_area if union_area > 0 else 0
    return iou

def listener():
    rospy.init_node('box_plotter', anonymous=True)
    rospy.Subscriber("/gazebo/ground_truth/box_markers", MarkerArray, callback_ground_truth)
    rospy.Subscriber("/bbox_markers", MarkerArray, callback_predicted)
    update_plot()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
