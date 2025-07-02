import cv2

def histogram_equalization(input_path, output_path):
    # 读取灰度图像
    image = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print("无法读取图像，请检查输入路径是否正确。")
        return

    # 直方图均衡化
    equalized_image = cv2.equalizeHist(image)

    # 保存均衡化后的图像
    cv2.imwrite(output_path, equalized_image)
    print(f"均衡化后的图像已保存到: {output_path}")

if __name__ == "__main__":
    # 用户输入图像路径和保存路径
    input_path = input("请输入灰度图像的路径: ")
    output_path = input("请输入均衡化后图像的保存路径: ")

    histogram_equalization(input_path, output_path)