import os
import cv2
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC
from sklearn.metrics import classification_report, accuracy_score

# 加载数据集并预处理
def load_and_preprocess_data(dataset_dir="fire_dataset"):
    images = []
    labels = []

    fire_dir = os.path.join(dataset_dir, r"D:\image\_fire_dir")
    non_fire_dir = os.path.join(dataset_dir, r"D:\image\_non_fire_dir")

    for label, folder in enumerate([fire_dir, non_fire_dir]):
        for file in os.listdir(folder):
            file_path = os.path.join(folder, file)
            img = cv2.imread(file_path)
            if img is not None:
                img = cv2.resize(img, (64, 64))  # 调整图像大小
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 转为灰度图
                images.append(img.flatten())  # 展平图像
                labels.append(label)

    images = np.array(images)
    labels = np.array(labels)
    return images, labels

# 添加函数：对单张图片进行预测
def predict_single_image(svm, image_path):
    if not os.path.exists(image_path):
        print("Error: 图片路径不存在。")
        return

    img = cv2.imread(image_path)
    if img is None:
        print("Error: Unable to read the image.")
        return

    # 预处理图片
    img = cv2.resize(img, (64, 64))  # 调整图像大小
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 转为灰度图
    img_flatten = img.flatten().reshape(1, -1)  # 展平并调整为二维数组

    # 预测火焰的概率
    probability = svm.decision_function(img_flatten)  # 使用 decision_function
    prediction = svm.predict(img_flatten)  # 获取预测类别

    print(f"Prediction: {'Fire' if prediction[0] == 0 else 'Non-Fire'}")
    print(f"Decision Function Value (Higher = More Confident): {probability[0]}")

# 主函数
def main():
    # 加载数据集并训练模型
    dataset_dir = r"D:\image"  # 替换为你的数据集文件夹路径
    images, labels = load_and_preprocess_data(dataset_dir)

    # 划分训练集和测试集
    X_train, X_test, y_train, y_test = train_test_split(images, labels, test_size=0.2, random_state=42)

    # 训练SVM模型
    print("Training SVM model...")
    svm = SVC(kernel='linear', random_state=42)
    svm.fit(X_train, y_train)

    # 测试模型
    print("Evaluating model...")
    y_pred = svm.predict(X_test)
    print("Accuracy:", accuracy_score(y_test, y_pred))
    print("Classification Report:\n", classification_report(y_test, y_pred))

    # 对单张图片进行预测
    test_image_path = r"D:\image\f4d97e3a_E865026_6256ba6d.jpg"  # 替换为你的测试图片路径
    print("\nPredicting single image...")
    predict_single_image(svm, test_image_path)

if __name__ == "__main__":
    main()