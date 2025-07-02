import numpy as np
import os
import matplotlib.pyplot as plt
from PIL import Image  # 用于加载和处理图片
import pywt  # 用于小波变换

def compute_dwt(image):
    """
    Compute the Discrete Wavelet Transform (DWT) of a 256x256 image.
    :param image: A 256x256 numpy array representing the image.
    :return: The approximation and detail coefficients of the DWT.
    """
    if image.shape != (256, 256):
        raise ValueError("Input image must be of size 256x256.")
    
    # 使用 Haar 小波进行 2D DWT
    coeffs2 = pywt.dwt2(image, 'haar')
    cA, (cH, cV, cD) = coeffs2  # 分别为近似系数和水平、垂直、对角细节系数
    return cA, cH, cV, cD

def load_and_process_image():
    """
    Allow the user to select an image file, resize it to 256x256 if necessary, and compute its DWT.
    """
    file_path = input("请输入图片文件的路径: ")
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"文件 {file_path} 不存在。")
    
    # 加载图片为灰度图
    try:
        image = Image.open(file_path).convert("L")  # 转为灰度图
    except Exception as e:
        raise ValueError(f"无法加载图片，请检查文件格式是否正确。错误: {e}")
    
    # 检查图像大小并调整为 256x256
    if image.size != (256, 256):
        print(f"图像大小为 {image.size}，将调整为 256x256。")
        image = image.resize((256, 256))
    
    # 转换为 numpy 数组
    image_array = np.array(image)
    
    # 计算 DWT
    cA, cH, cV, cD = compute_dwt(image_array)
    print("DWT 计算完成。")
    
    # 显示结果
    plt.figure(figsize=(10, 10))
    plt.subplot(2, 2, 1)
    plt.imshow(cA, cmap='gray')
    plt.title("Approximation (cA)")
    plt.colorbar()

    plt.subplot(2, 2, 2)
    plt.imshow(cH, cmap='gray')
    plt.title("Horizontal Detail (cH)")
    plt.colorbar()

    plt.subplot(2, 2, 3)
    plt.imshow(cV, cmap='gray')
    plt.title("Vertical Detail (cV)")
    plt.colorbar()

    plt.subplot(2, 2, 4)
    plt.imshow(cD, cmap='gray')
    plt.title("Diagonal Detail (cD)")
    plt.colorbar()

    plt.tight_layout()
    plt.show()
    
    return cA, cH, cV, cD

# 示例调用
if __name__ == "__main__":
    try:
        result = load_and_process_image()
        print("DWT 结果已成功生成。")
    except Exception as e:
        print(f"发生错误: {e}")