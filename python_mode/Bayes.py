import numpy as np
import matplotlib.pyplot as plt

data = np.array([-2.67,-3.55,-1.24,-0.98,-0.79,-2.85,-2.76,-3.73,-3.54,-2.27,-3.45,-3.08,-1.58,-1.49,-0.74,-0.42,-1.12,4.25,-3.99,2.88,-0.98])

def function_norma(x,u,sd):
    Fx = np.exp(-(x-u)**2/(2*sd**2))/(sd*np.sqrt(2*np.pi))
    return Fx


pw1 = 0.9
pw2 = 0.1
pxw1 = function_norma(data, -2, 1.5)
pxw2 = function_norma(data, -2, 2)
px = pxw1 * pw1 + pxw2 * pw2

new_array = np.zeros_like(data, dtype=int)
for i in range(len(data)):
    if pxw1[i] * pw1 > pxw2[i] * pw2:
        new_array[i] = 1
    else:
        new_array[i] = 2

print("第一类的内条件概率")
print(pxw1)
print("第二类的内条件概率")
print(pxw2)
print("基于最小错误率的贝叶斯分布结果：")
print("1表示第一类，2表示第二类")
print(new_array)

X_cool = np.arange(1, len(data) + 1)
plt.plot(X_cool, data, label='data')
plt.scatter(X_cool, new_array, s=20, c='r', label='class')
plt.legend()
plt.show()