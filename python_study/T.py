import numpy as np
import cv2
import os

def iterative_global_threshold(image, epsilon=1e-3):
    """
    实现迭代全局阈值算法
    :param image: 输入的灰度图像，例如 r"C:\\Users\\luo\\Pictures\\Screenshots\\hjkhkh.png"
    :param epsilon: 收敛条件，默认值为 1e-3
    :return: 计算得到的全局阈值
    """
    # 初始化阈值为图像的平均灰度值
    T = np.mean(image)
    
    while True:
        # 分割前景和背景
        foreground = image[image > T]
        background = image[image <= T]
        
        # 计算前景和背景的平均值
        mu1 = np.mean(foreground) if len(foreground) > 0 else 0
        mu2 = np.mean(background) if len(background) > 0 else 0
        
        # 计算新阈值
        T_new = (mu1 + mu2) / 2
        
        # 检查收敛性
        if abs(T_new - T) < epsilon:
            break
        
        T = T_new
    
    return T

def main():
    # 用户输入图像路径
    image_path = input("请输入图像路径：")
    
    # 读取图像
    image = cv2.imread(image_path)
    if image is None:
        print("无法读取图像，请检查路径是否正确！")
        return
    
    # 转换为灰度图像
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # 应用迭代全局阈值算法
    threshold = iterative_global_threshold(gray_image)
    print(f"计算得到的全局阈值为：{threshold}")
    
    # 使用阈值进行二值化
    _, binary_image = cv2.threshold(gray_image, threshold, 255, cv2.THRESH_BINARY)
    
    # 显示结果
    #cv2.imshow('Original Image', image)
    #cv2.imshow('Binary Image', binary_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 用户输入保存文件夹路径
    save_folder = input("请输入保存处理后图像的文件夹路径：")
    
    # 检查文件夹是否存在，不存在则创建
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
    
    # 保存二值化图像
    save_path = os.path.join(save_folder, "binary_image.png")
    cv2.imwrite(save_path, binary_image)
    print(f"处理后的图像已保存到：{save_path}")

if __name__ == "__main__":
    main()