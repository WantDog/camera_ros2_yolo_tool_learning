import cv2
import os
import tkinter as tk
from tkinter import messagebox, simpledialog, filedialog
import torch
from pathlib import Path
from ultralytics import YOLO

class VideoProcessor:
    """
    A class to handle video recording, frame extraction and image renaming operations.
    """
    def __init__(self):
        """Initialize paths and settings."""
        self.video_save_path = "/home/lzy/Desktop/数据集处理/videos"  # Path to save recorded videos
        self.image_save_path = "frames/"  # Path to save extracted frames
        self.camera_id = 2  # Default camera ID

        # Create directories if they don't exist
        os.makedirs(self.video_save_path, exist_ok=True)
        os.makedirs(self.image_save_path, exist_ok=True)

    def get_next_video_number(self) -> int:
        """Get the next available video number."""
        existing_videos = [f for f in os.listdir(self.video_save_path) if f.endswith('.mp4')]
        if not existing_videos:
            return 1
        numbers = [int(f.split('.')[0]) for f in existing_videos]
        return max(numbers) + 1

    def record_video(self):
        """Record video with keyboard controls."""
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            messagebox.showerror("Error", "Cannot open camera")
            return

        # Set camera resolution
        target_width = 1280
        target_height = 720
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
        
        # Verify the actual resolution
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera Resolution: {actual_width}x{actual_height}")
        # 常见相机的分辨率包括 640x480, 1280x720, 1920x1080 等
        out = None
        recording = False

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Show original frame without text
            cv2.imshow('Video Recording', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('a') and not recording:  # Start recording
                video_number = self.get_next_video_number()
                video_name = f"{video_number}.mp4"
                # 使用 avc1 编码器，确保与FFMPEG兼容
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                out = cv2.VideoWriter(os.path.join(self.video_save_path, video_name),
                                    fourcc, 20.0,  
                                    (actual_width, actual_height),
                                    isColor=True)
                if not out.isOpened():
                    print("Failed to create VideoWriter")
                    messagebox.showerror("Error", "Failed to create video file")
                    continue
                recording = True
                print(f"Started recording: {video_name}")
            elif key == ord('s') and recording:  # Stop and save recording
                out.release()
                out = None
                recording = False
                messagebox.showinfo("Success", "Video saved successfully!")
                print("Recording stopped and saved")
            elif key == ord('q'):  # Quit recording mode
                if recording and out is not None:
                    out.release()
                break

            if recording and out is not None:
                out.write(frame)  # Save original frame

        cap.release()
        cv2.destroyAllWindows()

    def get_next_frame_number(self) -> int:
        """Get the next available frame number."""
        existing_frames = [f for f in os.listdir(self.image_save_path) 
                          if f.endswith(('.jpg', '.jpeg', '.png'))]
        if not existing_frames:
            return 1
        try:
            numbers = [int(f.split('.')[0]) for f in existing_frames]
            return max(numbers) + 1
        except ValueError:
            # 如果文件名解析失败，返回1
            return 1

    def extract_frames(self):
        """Extract frames from a selected video."""
        while True:  # Allow multiple video selections
            # List available videos
            videos = [f for f in os.listdir(self.video_save_path) if f.endswith('.mp4')]
            if not videos:
                messagebox.showerror("Error", "No videos found!")
                return

            print("\nAvailable videos:")
            for video in sorted(videos):  # 排序显示视频列表
                print(f"- {video}")

            # Get video selection
            video_number = simpledialog.askstring("Input", "Enter video number (or 'q' to quit):")
            if not video_number or video_number.lower() == 'q':
                break
            if not video_number.isdigit():
                continue

            video_path = os.path.join(self.video_save_path, f"{video_number}.mp4")
            print(f"Attempting to open video at: {video_path}")
            
            if not os.path.exists(video_path):
                messagebox.showerror("Error", f"Video {video_number}.mp4 not found!")
                continue

            try:
                # 强制使用FFMPEG后端
                cap = cv2.VideoCapture(str(video_path), cv2.CAP_FFMPEG)
                if not cap.isOpened():
                    print(f"Failed to open video: {video_path}")
                    messagebox.showerror("Error", f"Cannot open video {video_number}.mp4")
                    continue

                # 打印视频信息
                total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                fps = int(cap.get(cv2.CAP_PROP_FPS))
                print(f"Video info - Total frames: {total_frames}, FPS: {fps}")

                # Get frame interval
                interval = simpledialog.askinteger("Input", "Enter frame interval:")
                if not interval or interval < 1:
                    cap.release()
                    continue

                frame_count = 0
                next_frame_number = self.get_next_frame_number()
                frames_extracted = 0

                while True:
                    ret, frame = cap.read()
                    if not ret:
                        break

                    if frame_count % interval == 0:
                        save_path = os.path.join(self.image_save_path, f"{next_frame_number}.jpg")
                        cv2.imwrite(save_path, frame)
                        next_frame_number += 1
                        frames_extracted += 1
                        print(f"Extracted frame {next_frame_number-1}")  # 添加进度提示

                    frame_count += 1

                cap.release()
                
                if frames_extracted > 0:
                    messagebox.showinfo("Success", 
                                      f"Extracted {frames_extracted} frames from video {video_number}.mp4\n"
                                      f"Frames saved as {next_frame_number-frames_extracted}.jpg to {next_frame_number-1}.jpg")
                else:
                    messagebox.showwarning("Warning", "No frames were extracted!")

            except Exception as e:
                print(f"Error processing video: {str(e)}")
                messagebox.showerror("Error", f"Error processing video: {str(e)}")
                if 'cap' in locals():
                    cap.release()

    def rename_frames(self):
        """Rename all frames in the image directory to sequential numbers."""
        frames = [f for f in os.listdir(self.image_save_path) if f.endswith(('.jpg', '.jpeg', '.png'))]
        if not frames:
            messagebox.showinfo("Info", "No frames found to rename!")
            return
        
        frames.sort()  # Sort files to maintain order
        
        # Ask for starting number
        start_number = simpledialog.askinteger("Input", "Enter starting number (default: 1):", 
                                             initialvalue=1)
        if start_number is None:  # User cancelled
            return
        if start_number < 1:
            messagebox.showerror("Error", "Starting number must be positive!")
            return

        # Temporary rename to avoid conflicts
        for i, frame in enumerate(frames):
            old_path = os.path.join(self.image_save_path, frame)
            temp_path = os.path.join(self.image_save_path, f"temp_{i + 1}.jpg")
            os.rename(old_path, temp_path)

        # Rename to final names with specified starting number
        for i in range(len(frames)):
            temp_path = os.path.join(self.image_save_path, f"temp_{i + 1}.jpg")
            new_path = os.path.join(self.image_save_path, f"{start_number + i}.jpg")
            os.rename(temp_path, new_path)

        messagebox.showinfo("Success", 
                           f"Renamed {len(frames)} frames starting from {start_number}.jpg!")

    def delete_unpaired_labels(self):
        """Delete label files that don't have corresponding images."""
        # Get images directory
        images_dir = simpledialog.askstring("Input", "Enter images directory path:")
        if not images_dir or not os.path.isdir(images_dir):
            messagebox.showerror("Error", "Invalid images directory path!")
            return

        # Get labels directory
        labels_dir = simpledialog.askstring("Input", "Enter labels directory path:")
        if not labels_dir or not os.path.isdir(labels_dir):
            messagebox.showerror("Error", "Invalid labels directory path!")
            return

        # Get image names without extensions
        image_names = {os.path.splitext(f)[0] for f in os.listdir(images_dir) 
                      if f.lower().endswith(('.jpg', '.jpeg', '.png'))}
        
        # Get label names without extensions
        label_files = [f for f in os.listdir(labels_dir) if f.lower().endswith('.txt')]
        deleted_count = 0

        # Delete unpaired label files
        for label_file in label_files:
            label_name = os.path.splitext(label_file)[0]
            if label_name not in image_names:
                try:
                    os.remove(os.path.join(labels_dir, label_file))
                    deleted_count += 1
                    print(f"Deleted: {label_file}")
                except Exception as e:
                    print(f"Error deleting {label_file}: {str(e)}")

        if deleted_count > 0:
            messagebox.showinfo("Success", f"Deleted {deleted_count} unpaired label files!")
        else:
            messagebox.showinfo("Info", "No unpaired label files found!")

    def label_images(self):
        """
        Label images using a YOLO model.
        Detects objects in images and saves results to text files in YOLO format.
        """
        # Get images directory
        images_dir = filedialog.askdirectory(title="Select Images Directory")
        if not images_dir or not os.path.isdir(images_dir):
            messagebox.showerror("Error", "Invalid images directory path!")
            return

        # Get labels directory
        labels_dir = filedialog.askdirectory(title="Select Labels Directory")
        if not labels_dir or not os.path.isdir(labels_dir):
            messagebox.showerror("Error", "Invalid labels directory path!")
            return
            
        # Get model file path
        model_path = filedialog.askopenfilename(
            title="Select Model File",
            filetypes=[("PyTorch Models", "*.pt")]
        )
        if not model_path or not os.path.isfile(model_path):
            messagebox.showerror("Error", "Invalid model file path!")
            return
            
        try:
            # Create labels directory if it doesn't exist
            os.makedirs(labels_dir, exist_ok=True)
            
            # Load YOLO model using Ultralytics
            model = YOLO(model_path)
            
            # Get all image files
            image_files = [f for f in os.listdir(images_dir) 
                          if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
            
            if not image_files:
                messagebox.showinfo("Info", "No images found in selected directory!")
                return
                
            # Process each image
            processed_count = 0
            for img_file in image_files:
                img_path = os.path.join(images_dir, img_file)
                base_name = os.path.splitext(img_file)[0]
                label_path = os.path.join(labels_dir, f"{base_name}.txt")
                
                # Run inference with high confidence
                results = model(img_path, conf=0.25)
                
                # Get image dimensions for normalization
                img = cv2.imread(img_path)
                img_height, img_width = img.shape[:2]
                
                # Create and save label file
                with open(label_path, 'w') as f:
                    # Process detections from the first image result
                    if len(results) > 0:
                        boxes = results[0].boxes
                        for box in boxes:
                            # Get class ID
                            class_id = int(box.cls.item())
                            
                            # Get bounding box coordinates (already normalized in xyxy format)
                            xyxy = box.xyxy[0].tolist()
                            x1, y1, x2, y2 = xyxy
                            
                            # Convert to YOLO format (normalized center x, center y, width, height)
                            x_center = ((x1 + x2) / 2) / img_width
                            y_center = ((y1 + y2) / 2) / img_height
                            width = (x2 - x1) / img_width
                            height = (y2 - y1) / img_height
                            
                            # Write to file in YOLO format
                            f.write(f"{class_id} {x_center} {y_center} {width} {height}\n")
                
                processed_count += 1
                print(f"Processed: {img_file} -> {base_name}.txt")
            
            messagebox.showinfo("Success", f"Processed {processed_count} images and saved labels!")
            
        except Exception as e:
            print(f"Error during labeling: {str(e)}")
            messagebox.showerror("Error", f"Error during labeling: {str(e)}")

class GUI:
    """GUI for the video processing application."""
    def __init__(self):
        self.processor = VideoProcessor()
        self.root = tk.Tk()
        self.root.title("Video Processing Tool")
        self.setup_gui()

    def setup_gui(self):
        """Set up the GUI elements."""
        tk.Label(self.root, text="Select Mode:", font=('Arial', 14)).pack(pady=10)
        
        tk.Button(self.root, text="1. Record Video", 
                 command=self.processor.record_video).pack(pady=5)
        tk.Button(self.root, text="2. Extract Frames", 
                 command=self.processor.extract_frames).pack(pady=5)
        tk.Button(self.root, text="3. Rename Frames", 
                 command=self.processor.rename_frames).pack(pady=5)
        tk.Button(self.root, text="4. Delete Unpaired Labels", 
                 command=self.processor.delete_unpaired_labels).pack(pady=5)
        tk.Button(self.root, text="5. Labeling", 
                 command=self.processor.label_images).pack(pady=5)
        tk.Button(self.root, text="Exit", 
                 command=self.root.quit).pack(pady=10)

    def run(self):
        """Start the GUI application."""
        self.root.mainloop()

def main():
    """Main function to run the application."""
    app = GUI()
    app.run()

if __name__ == "__main__":
    main()
