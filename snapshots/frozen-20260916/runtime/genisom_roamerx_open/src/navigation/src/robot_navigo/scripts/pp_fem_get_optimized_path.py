import matplotlib.pyplot as plt
import csv
import glob
import os

def find_workspace_dir(target_folder_name="jszr_workspace"):
    current_path = os.path.abspath(__file__)  
    current_dir = os.path.dirname(current_path)

    while True:
        if os.path.basename(current_dir) == target_folder_name:
            return current_dir
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:
            raise RuntimeError(f"未找到名为 '{target_folder_name}' 的目录")
        current_dir = parent_dir

DATA_DIR = find_workspace_dir("jszr_workspace")
print(f"自动识别到工程根目录: {DATA_DIR}")

def load_csv(file_path):
    x, y, theta = [], [], []
    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            x.append(float(row['x']))
            y.append(float(row['y']))
            theta.append(float(row['theta']))
    return x, y, theta

def draw_segments(prefix, color='b', linestyle='-', linewidth=1.5, label_prefix=''):
    pattern = os.path.join(DATA_DIR, f"{prefix}_segment_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"⚠️ No files found for pattern: {pattern}")
        return

    for i, f in enumerate(files):
        x, y, _ = load_csv(f)
        # 只对第一个 segment 加入图例标签，避免 legend 混乱
        label = f"{label_prefix}" if i == 0 else None
        plt.plot(x, y, linestyle=linestyle, color=color, linewidth=linewidth, label=label)

plt.figure(figsize=(10, 8))

draw_segments("raw_path", color='#FF5733', linestyle='--', linewidth=2.5, label_prefix='Raw')
draw_segments("pp_path", color='#1f77b4', linestyle='-.', linewidth=2.5, label_prefix='PP')
draw_segments("pp_fem_path", color='#2ca02c', linestyle='-', linewidth=2.8, label_prefix='PP+FEM')

key_pose_file = os.path.join(DATA_DIR, "key_poses.csv")
key_x, key_y, _ = load_csv(key_pose_file)

# 关键点绘制
plt.scatter(key_x, key_y, c='red', marker='o', s=150, edgecolors='black', linewidths=1.5, label="Key Poses")

# 关键点序号标注
for i, (x, y) in enumerate(zip(key_x, key_y), start=1):
    plt.text(x + 0.05, y + 0.05, f'{i}', fontsize=12, color='black', weight='bold')

plt.title("All Path Segments Optimization Comparison", fontsize=14)
plt.xlabel("X [m]", fontsize=12)
plt.ylabel("Y [m]", fontsize=12)
plt.axis("equal")
plt.grid(True)
plt.legend(fontsize=10)
plt.tight_layout()
plt.show()
