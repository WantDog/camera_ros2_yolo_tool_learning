import cv2
import time
import os

cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# === 创建 videos 目录 ===
output_dir = "videos"
os.makedirs(output_dir, exist_ok=True)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps_camera = cap.get(cv2.CAP_PROP_FPS)
if fps_camera == 0:
    fps_camera = 30

fourcc = cv2.VideoWriter_fourcc(*'XVID') # type: ignore

recording = False
video_writer = None
video_count = 0

# ---------- FPS 统计 ----------
frame_count = 0
fps = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ---------- 统计帧数 ----------
    frame_count += 1
    elapsed = time.time() - start_time

    if elapsed >= 1.0:
        fps = frame_count
        frame_count = 0
        start_time = time.time()

    # ---------- 显示 FPS ----------
    cv2.putText(
        frame,
        f"FPS: {fps}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    # ---------- 录制 ----------
    if recording and video_writer is not None:
        video_writer.write(frame)
        cv2.putText(
            frame,
            "REC",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('r'):
        if not recording:
            video_count += 1
            # 修改这里：将视频保存到 videos 文件夹
            filename = os.path.join(output_dir, f"video_{video_count}.avi")
            video_writer = cv2.VideoWriter(
                filename, fourcc, fps_camera, (width, height)
            )
            recording = True
            print(f"开始录制：{filename}")
        else:
            recording = False
            if video_writer is not None:
                video_writer.release()
                video_writer = None
            print("停止录制")

    elif key == ord('q'):
        break

if video_writer is not None:
    video_writer.release()

cap.release()
cv2.destroyAllWindows()