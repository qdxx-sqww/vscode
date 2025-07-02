import cv2
import numpy as np

def hough_transform_line_detection():
    # 用户输入图像路径
    image_path = input("请输入图像路径：")
    save_path = input("请输入结果保存路径：")

    # 读取图像
    image = cv2.imread(image_path)
    if image is None:
        print("无法读取图像，请检查路径是否正确。")
        return

    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 边缘检测（Canny 算法）
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # 霍夫变换检测直线
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

    # 绘制检测到的直线
    if lines is not None:
        for rho, theta in lines[:, 0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # 保存结果图像
    cv2.imwrite(save_path, image)
    print(f"结果已保存到 {save_path}")

if __name__ == "__main__":
    hough_transform_line_detection()