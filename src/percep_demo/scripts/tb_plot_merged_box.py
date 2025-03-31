import json
import matplotlib.pyplot as plt

# 读取 JSON 文件
json_file = 'merged_bboxes.json'  # 请将此处替换为你的 JSON 文件路径
with open(json_file, 'r') as f:
    data = json.load(f)

# 创建绘图
plt.figure(figsize=(10, 10))

# 提取所有 bbox 信息
bboxes = data['bboxes']

# 遍历并打印每个 bbox 的信息
for idx, bbox in enumerate(bboxes):
    # 提取中心点坐标 (忽略 z 轴)
    type = bbox['type']
    center_x, center_y = bbox['center'][0], bbox['center'][1]
    # 提取 min_bound 和 max_bound 坐标 (忽略 z 轴)
    min_x, min_y = bbox['min_bound'][0], bbox['min_bound'][1]
    max_x, max_y = bbox['max_bound'][0], bbox['max_bound'][1]

    if type == "box":
        # 绘制蓝色方框
        plt.plot([min_x, max_x], [min_y, min_y], 'b-')  # 底边
        plt.plot([min_x, max_x], [max_y, max_y], 'b-')  # 顶边
        plt.plot([min_x, min_x], [min_y, max_y], 'b-')  # 左边
        plt.plot([max_x, max_x], [min_y, max_y], 'b-')  # 右边

        # 绘制红色 X 标记中心点
        plt.plot(center_x, center_y, 'rx')
    
    if type == "bridge":
        # 绘制灰色方框
        plt.plot([min_x, max_x], [min_y, min_y], 'grey', '-')  # 底边
        plt.plot([min_x, max_x], [max_y, max_y], 'grey', '-')  # 顶边
        plt.plot([min_x, min_x], [min_y, max_y], 'grey', '-')  # 左边
        plt.plot([max_x, max_x], [min_y, max_y], 'grey', '-')  # 右边

    if type == "known":
        # 绘制橘色方框
        plt.plot([min_x, max_x], [min_y, min_y], 'orange', '-')  # 底边
        plt.plot([min_x, max_x], [max_y, max_y], 'orange', '-')  # 顶边
        plt.plot([min_x, min_x], [min_y, max_y], 'orange', '-')  # 左边
        plt.plot([max_x, max_x], [min_y, max_y], 'orange', '-')  # 右边

wall_min_x = 1.57
wall_max_x = 23.4
wall_min_y = 8.5
wall_max_y = 20.4
plt.plot([wall_min_x, wall_max_x], [wall_min_y, wall_min_y], 'g-')  # 底边
plt.plot([wall_min_x, wall_max_x], [wall_max_y, wall_max_y], 'g-')  # 顶边
plt.plot([wall_min_x, wall_min_x], [wall_min_y, wall_max_y], 'g-')  # 左边
plt.plot([wall_max_x, wall_max_x], [wall_min_y, wall_max_y], 'g-')  # 右边
plt.plot(7.3, 18.25, 'kx')  # 黑色
plt.plot(5.6, 16.5, 'kx')
plt.plot(4.0, 13.9, 'kx')
plt.plot(7.5, 13.3, 'kx')
plt.plot(11.1, 14.6, 'kx')
plt.plot(11.2, 12.7, 'kx')
plt.plot(14.0, 17.2, 'kx')
plt.plot(12.9, 15.9, 'kx')
plt.plot(14.6, 15.7, 'kx')
plt.plot(14.5, 13.8, 'kx')
plt.plot(14.4, 11.5, 'kx')
plt.plot(20.9, 15.6, 'kx')
plt.plot(21.0, 11.6, 'kx')

plt.plot(14.5, 9.0, 'kx')
plt.plot(16.0, 9.0, 'kx')
plt.plot(15.3, 7.0, 'kx')

# 设置绘图属性
plt.title('2D Bounding Boxes Visualization')
plt.xlabel('X Axis')
plt.ylabel('Y Axis')
plt.grid(True)
plt.axis('equal')  # 保证比例一致
plt.show()
