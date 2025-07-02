import numpy as np
import matplotlib.pyplot as plt

# 数据准备
x1 = np.array([0.23, 1.52, 0.65, 0.77, 1.05, 1.19, 0.29, 0.25, 0.66, 0.56, 0.90, 0.13, -0.54, 0.94, -0.21, 0.05, -0.08, 0.73, 0.33, 1.06, -0.02, 0.11, 0.31, 0.66])
y1 = np.array([2.34, 2.19, 1.67, 1.63, 1.78, 2.01, 2.06, 2.12, 2.47, 1.51, 1.96, 1.83, 1.87, 2.29, 1.77, 2.39, 1.56, 1.93, 2.20, 2.45, 1.75, 1.69, 2.48, 1.72])
x2 = np.array([1.40, 1.23, 2.08, 1.16, 1.37, 1.18, 1.76, 1.97, 2.41, 2.58, 2.84, 1.95, 1.25, 1.28, 1.26, 2.01, 2.18, 1.79, 1.33, 1.15, 1.70, 1.59, 2.93, 1.46])
y2 = np.array([1.02, 0.96, 0.91, 1.49, 0.82, 0.93, 1.14, 1.06, 0.81, 1.28, 1.46, 1.43, 0.71, 1.29, 1.37, 0.93, 1.22, 1.18, 0.87, 0.55, 0.51, 0.99, 0.91, 0.71])

# 合并数据，w1为正类+1，w2为负类-1
X = np.vstack((np.column_stack((x1, y1)), np.column_stack((x2, y2))))
y = np.hstack((np.ones(len(x1)), -np.ones(len(x2))))

# 感知器算法
w = np.array([1.0, 1.0])  # 初始权向量
b = 1.0                   # 初始偏置
lr = 1.0                  # 学习率
max_iter = 1000
iter_count = 0

for _ in range(max_iter):
    error_count = 0
    for xi, yi in zip(X, y):
        if yi * (np.dot(w, xi) + b) <= 0:
            w += lr * yi * xi
            b += lr * yi
            error_count += 1
    iter_count += 1
    if error_count == 0:
        break

print(f"感知器收敛，迭代次数：{iter_count}")
print(f"最终权向量 w: {w}, 偏置 b: {b}")

# 分类判断
test_points = np.array([
    [1.0, 1.5],
    [1.2, 1.0],
    [2.0, 0.9],
    [1.2, 1.5],
    [0.23, 2.33]
])
test_results = []
for pt in test_points:
    res = np.dot(w, pt) + b
    label = 1 if res > 0 else -1
    test_results.append(label)
    print(f"点{tuple(pt)}属于类别：{'w1' if label == 1 else 'w2'}")

# 可视化
plt.figure(figsize=(8,6))
plt.scatter(x1, y1, c='b', label='w1类')
plt.scatter(x2, y2, c='r', label='w2类')
plt.scatter(test_points[:,0], test_points[:,1], c='g', marker='*', s=150, label='测试点')

# 画分界线
x_plot = np.linspace(-1, 3, 100)
y_plot = -(w[0]*x_plot + b)/w[1]
plt.plot(x_plot, y_plot, 'k--', label='感知器分界线')

plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title('感知器分类结果')
plt.grid(True)
plt.show()