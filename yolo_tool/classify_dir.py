import os
import shutil

# 设置你的文件路径
source_dir = './picture'  # 你的图片和标签文件所在的目录
image_dir = './images/'  # 图片文件保存的目录
label_dir = './labels/'  # 标签文件保存的目录

# 创建目录（如果不存在）
os.makedirs(image_dir, exist_ok=True)
os.makedirs(label_dir, exist_ok=True)

# 获取目录下的所有文件
files = os.listdir(source_dir)

# 遍历所有文件
for file in files:
    # 判断文件是图片还是标签
    if file.endswith('.jpg'):
        # 如果是图片文件，移动到 images 文件夹
        shutil.move(os.path.join(source_dir, file), os.path.join(image_dir, file))
    elif file.endswith('.txt'):
        # 如果是标签文件，移动到 labels 文件夹
        shutil.move(os.path.join(source_dir, file), os.path.join(label_dir, file))

print("文件已分离到相应的文件夹。")
