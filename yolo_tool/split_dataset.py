import os
import random
import shutil

# ======================
# 可调整参数
# ======================
# 数据路径
images_dir = 'images'
labels_dir = 'labels'

# 输出路径
output_base = 'dataset'   # 输出目录，会生成 dataset/train 等

# 划分比例（总和为1）
train_ratio = 0.7
valid_ratio = 0.2
test_ratio = 0.1

# ======================
# 程序开始
# ======================
def make_dirs(base):
    for split in ['train', 'valid', 'test']:
        os.makedirs(os.path.join(base, split, 'images'), exist_ok=True)
        os.makedirs(os.path.join(base, split, 'labels'), exist_ok=True)

def copy_files(file_list, split_name):
    for img_file in file_list:
        base_name = os.path.splitext(os.path.basename(img_file))[0]
        label_file = os.path.join(labels_dir, base_name + '.txt')

        # 拷贝图像
        shutil.copy(img_file, os.path.join(output_base, split_name, 'images', os.path.basename(img_file)))

        # 拷贝对应标签
        if os.path.exists(label_file):
            shutil.copy(label_file, os.path.join(output_base, split_name, 'labels', os.path.basename(label_file)))
        else:
            print(f"⚠️ 标签缺失: {label_file}")

def main():
    # 获取所有图片路径
    image_files = [os.path.join(images_dir, f) for f in os.listdir(images_dir) 
                   if f.endswith(('.jpg', '.png', '.jpeg'))]
    image_files.sort()
    random.shuffle(image_files)

    total = len(image_files)
    n_train = int(total * train_ratio)
    n_valid = int(total * valid_ratio)
    # 测试集自动补齐剩下的
    n_test = total - n_train - n_valid

    print(f"总图片数: {total}")
    print(f"训练集: {n_train}  验证集: {n_valid}  测试集: {n_test}")

    train_files = image_files[:n_train]
    valid_files = image_files[n_train:n_train+n_valid]
    test_files = image_files[n_train+n_valid:]

    make_dirs(output_base)

    copy_files(train_files, 'train')
    copy_files(valid_files, 'valid')
    copy_files(test_files, 'test')

    print("✅ 数据集划分完成！输出路径:", os.path.abspath(output_base))

if __name__ == "__main__":
    main()
