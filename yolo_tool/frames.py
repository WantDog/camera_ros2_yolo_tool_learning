import cv2
import os

# === 参数设置 ===
video_path = ""          # 输入视频路径
save_dir = "images"               # 保存帧的文件夹
frame_interval = 6           # 每隔多少帧保存一张
start_index = 0         # 起始编号（从多少开始计数）

# === 创建输出目录 ===
os.makedirs(save_dir, exist_ok=True)

# === 打开视频 ===
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("无法打开视频！请检查路径是否正确。")
    exit()

# === 获取视频信息 ===
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
fps = cap.get(cv2.CAP_PROP_FPS)
print(f"视频总帧数: {total_frames}, 帧率: {fps}")

frame_count = 0
saved_count = 0
current_index = start_index  # 使用起始编号

# === 循环读取帧 ===
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 每隔 frame_interval 帧保存一次
    if frame_count % frame_interval == 0:
        filename = os.path.join(save_dir, f"frame_{current_index:05d}.jpg")
        cv2.imwrite(filename, frame)
        saved_count += 1
        current_index += 1  # 递增索引

    frame_count += 1

cap.release()
print(f"共提取 {saved_count} 张帧，保存在 '{save_dir}' 文件夹中。")
print(f"文件编号从 {start_index:05d} 开始")