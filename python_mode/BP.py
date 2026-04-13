import numpy as np
import tensorflow as tf

# 数据和标签
data_x = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype="float32")
data_y = np.array([[0], [1], [1], [1]], dtype="float32")

# 构建模型
x = tf.placeholder("float", [None, 2])
x = tf.keras.Input(shape=(2,), dtype=tf.float32)
y = tf.keras.Input(shape=(1,), dtype=tf.float32)
# 权重和偏置初始化
W1 = tf.Variable(tf.random_normal([2, 1], 0.1, 0.1))
W1 = tf.Variable(tf.random.normal([2, 1], mean=0.1, stddev=0.1))
b1 = tf.Variable(tf.random.normal([1], mean=0.1, stddev=0.1))
# 前向传播
z1 = tf.matmul(x, W1) + b1
y_pred = tf.nn.sigmoid(z1)

# 定义损失函数
loss_l2 = tf.reduce_sum(tf.square(y - y_pred))

# 优化器和训练步骤
lr = tf.placeholder(tf.float32)
train_step = tf.train.GradientDescentOptimizer(lr).minimize(loss_l2)

# 初始化变量
init = tf.global_variables_initializer()
init = tf.compat.v1.global_variables_initializer()
sess = tf.compat.v1.Session()
sess.run(init)
# 训练模型
errors = []
for epoch in range(5000):
    if epoch <= 2000:
        current_lr = 1
    elif epoch <= 4000:
        current_lr = 0.1
    else:
        current_lr = 0.01

    for i in range(4):
        batch_xs = data_x[i:i + 1]
        batch_ys = data_y[i:i + 1]
        _, loss, y_val = sess.run([train_step, loss_l2, y_pred], feed_dict={x: batch_xs, y: batch_ys, lr: current_lr})

    errors.append(loss)
    if (epoch + 1) % 500 == 0:
        print(f"Epoch: {epoch + 1} -> Loss: {loss:.4f} -> Pred: {y_val}")

# 定义预测函数
def predict(x_input):
    x_input = np.array(x_input, dtype="float32")
    assert x_input.shape[1] == 2, "Input shape must be (?, 2)"
    pred = sess.run(y_pred, feed_dict={x: x_input})
    return (pred > 0.5).astype(np.float32)

# 测试预测结果
test_input = [[0, 1]]
test_output = predict(test_input)
print(f"Test Input: {test_input} -> Prediction: {test_output}")