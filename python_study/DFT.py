import numpy as np
import os
import matplotlib.pyplot as plt
from PIL import Image  # 用于加载和处理图片

def compute_dft(image):
    """
    Compute the Discrete Fourier Transform (DFT) of a 256x256 image.
    :param image: A 256x256 numpy array representing the image.
    :return: The DFT of the image.
    """
    if image.shape != (256, 256):
        raise ValueError("Input image must be of size 256x256.")
    dft = np.fft.fft2(image)
    dft_shifted = np.fft.fftshift(dft)  # Shift the zero frequency component to the center
    magnitude_spectrum = 20 * np.log(np.abs(dft_shifted) + 1)  # Compute magnitude spectrum
    return magnitude_spectrum

def load_and_process_image():
    """
    Allow the user to select an image file, resize it to 256x256 if necessary, and compute its DFT.
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
    
    # 计算 DFT
    dft_result = compute_dft(image_array)
    print("DFT 计算完成。")
    
    # 显示结果
    plt.imshow(dft_result, cmap='gray')
    plt.title("Magnitude Spectrum")
    plt.colorbar()
    plt.show()
    
    return dft_result

# 示例调用
if __name__ == "__main__":
    try:
        result = load_and_process_image()
        print("DFT 结果已成功生成。")
    except Exception as e:
        print(f"发生错误: {e}")