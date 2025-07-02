import numpy as np
import matplotlib.pyplot as plt
import math
x1=[0.23,1.52,0.65,0.77,1.05,1.19,0.29,0.25,0.66,0.56,0.90,0.13,-0.54,0.94,-0.21,0.05,-0.08,0.73,0.33,1.06,-0.02,0.11,0.31,0.66]
y1=[2.34,2.19,1.67,1.63,1.78,2.01,2.06,2.12,2.47,1.51,1.96,1.83,1.87,2.29,1.77,2.39,1.56,1.93,2.20,2.45,1.75,1.69,2.48,1.72]
x2=[1.40,1.23,2.08,1.16,1.37,1.18,1.76,1.97,2.41,2.58,2.84,1.95,1.25,1.28,1.26,2.01,2.18,1.79,1.33,1.15,1.70,1.59,2.93,1.46]
y2=[1.02,0.96,0.91,1.49,0.82,0.93,1.14,1.06,0.81,1.28,1.46,1.43,0.71,1.29,1.37,0.93,1.22,1.18,0.87,0.55,0.51,0.99,0.91,0.71]
#整合x1,y1,x2,y2矩阵为w1,w2
w1 = np.column_stack((x1, y1))
w2 = np.column_stack((x2, y2))
print("整合矩阵w1 w2为:")
print(w1)
print(w2)
#计算两类均值向量
m1 = np.mean(w1, axis=0)
m2 = np.mean(w2, axis=0)
print("计算两类均值向量为：")
print(m1)
print(m2)
# 计算类内散度矩阵
s1 = np.zeros((2, 2))
s2 = np.zeros((2, 2))
for i in range(24):
    s1 += np.outer(w1[i] - m1, w1[i] - m1)
    s2 += np.outer(w2[i] - m2, w2[i] - m2)
print('s1')
print(s1)
print('s2')
print(s2)
sw = s1 + s2
print('sw')
print(sw)
# 计算投影方向和阈值
w_new = np.linalg.inv(sw).dot((m1 - m2).reshape(-1, 1))
print('w_new')
print(w_new)
m1_new = np.dot(m1, w_new)
m2_new = np.dot(m2, w_new)
pw1 = 0.6
pw2 = 0.4
w0=(m1_new+m2_new)/2-math.log(pw1/pw2)/(24+24-2)
print('w0')
print(w0)
#对测试数据进行分类判别
x=[[1,1.5],[1.2,1.0],[2.0,0.9],[1.2,1.5],[0.23,2.33]]
result1=[]
result2=[]
for i in range(5):
    y = np.dot(np.array(x[i]), w_new).item()  # 得到标量
    if y > w0:
        result1.append(x[i])
    else:
        result2.append(x[i])
print('result1')
print(result1)
print('result2')
print(result2)
# 计算试验点在w_new方向上的点
w_k = w_new / np.linalg.norm(w_new, ord=2)  # 归一化为单位向量，形状(2,1)
print(w_k)
wd = np.zeros((2, 5))
for i in range(5):
    # 投影点 = 原点 + (x到投影轴的投影长度) * 单位方向向量
    x_vec = np.array(x[i]).reshape(2, 1)
    proj_len = np.dot(w_k.T, x_vec)  # 投影长度，标量
    wd[:, i] = (proj_len * w_k).flatten()
print('wd')
print(wd)
# 显示分类结果
mw1 = w1
mw2 = w2
mr1 = np.array(result1)
mr2 = np.array(result2)
p1 = plt.scatter(mw1[:, 0], mw1[:, 1], c='red', marker='+')  # 画出w1类的各点
p2 = plt.scatter(mw2[:, 0], mw2[:, 1], c='green', marker='s')  # 画出w2类的各点
if mr1.size > 0:
    p3 = plt.scatter(mr1[:, 0], mr1[:, 1])  # 画出测试集中属于w1的各点
else:
    p3 = plt.scatter([], [])
if mr2.size > 0:
    p4 = plt.scatter(mr2[:, 0], mr2[:, 1])  # 画出测试集中属于w2的各点
else:
    p4 = plt.scatter([], [])
p5 = plt.plot([0, 10 * w_new[0][0]], [0, 10 * w_new[1][0]])  # 画出最佳投影方向
p6 = plt.scatter(wd[0, :], wd[1, :], c='g', marker='*')  # 画出测试集各点在投影方向上的投影点
plt.legend([p1, p2, p3, p4, p6], ['w1', 'w2', 'result1', 'result2', 'lx'])
plt.show()