import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.filters import prewitt, sobel, laplace
from skimage import io, color
from skimage.filters.edges import roberts

def edge_detection(image_path):
    # 读取图像
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        print("无法读取图像，请检查路径是否正确。")
        return

    # Canny 边缘检测
    edges_canny = cv2.Canny(image, 100, 200)

    # Prewitt 边缘检测
    prewitt_edges = prewitt(image)

    # Sobel 边缘检测
    sobel_edges = sobel(image)

    # Laplacian of Gaussian (LoG) 边缘检测
    log_edges = laplace(image)

    # Roberts 边缘检测
    roberts_edges = roberts(image)

    # 显示结果
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 3, 1)
    plt.imshow(image, cmap='gray')
    plt.title('原始图像')
    plt.axis('off')

    plt.subplot(2, 3, 2)
    plt.imshow(edges_canny, cmap='gray')
    plt.title('Canny 边缘检测')
    plt.axis('off')

    plt.subplot(2, 3, 3)
    plt.imshow(prewitt_edges, cmap='gray')
    plt.title('Prewitt 边缘检测')
    plt.axis('off')

    plt.subplot(2, 3, 4)
    plt.imshow(sobel_edges, cmap='gray')
    plt.title('Sobel 边缘检测')
    plt.axis('off')

    plt.subplot(2, 3, 5)
    plt.imshow(log_edges, cmap='gray')
    plt.title('LoG 边缘检测')
    plt.axis('off')

    plt.subplot(2, 3, 6)
    plt.imshow(roberts_edges, cmap='gray')
    plt.title('Roberts 边缘检测')
    plt.axis('off')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    image_path = input("请输入图像路径: ")
    edge_detection(image_path)