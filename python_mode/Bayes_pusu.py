import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.naive_bayes import GaussianNB
from sklearn.datasets import load_iris
from sklearn.metrics import recall_score
from sklearn.metrics import accuracy_score
from sklearn.metrics import precision_score
from sklearn.metrics import f1_score
import matplotlib.pyplot as plt
import seaborn as sns
sns.set_style('whitegrid')

#用sklearn自带数据集
iris_sklearn = load_iris(as_frame=True)
iris = iris_sklearn.frame
iris['Species'] = iris['target'].map(dict(enumerate(iris_sklearn.target_names)))

# 设置配色
palette = ['#1890FF','#2FC25B','#FACC14','#223273','#8543E0','#13C2C2','#3436c7','#F04864']
# 用seaborn生成散点图矩阵
sg = sns.pairplot(data=iris, palette=palette, hue='Species')
print(iris.info())
# 输出Iris数据集信息

palette = ['#1890FF','#2FC25B','#FACC14','#223273','#8543E0','#13C2C2','#3436c7','#F04864']
labelColorMapping = {}
def getColor(label):
    if label in labelColorMapping:
        return labelColorMapping[label]
    else:
        res = list(set(palette).difference(set(labelColorMapping.values())))
        if res:
            labelColorMapping[label] = res[0]
            return res[0]
        else:
            return '#000000'

def radar(data, title='Radar Chart'):
    plt.rcParams['font.sans-serif'] = 'Microsoft YaHei'
    plt.rcParams['axes.unicode_minus'] = False
    plt.style.use('ggplot')
    values = []
    labels = []
    feature = data.columns.values[0:4]
    for _, row in data.iterrows():
        values.append(row.tolist()[0:4] + [row.tolist()[0]])
        labels.append(row.tolist()[4])
    labels_kind = list(set(labels))
    N = len(feature)
    angles = np.linspace(0, 2 * np.pi, N, endpoint=False).tolist()
    angles += angles[:1]
    fig, ax = plt.subplots(ncols=len(labels_kind), nrows=1, subplot_kw=dict(polar=True))
    fig.tight_layout()
    # 修复：保证ax始终为可索引的list
    if len(labels_kind) == 1:
        ax = [ax]
    for value, label in zip(values, labels):
        plot_pos = labels_kind.index(label)
        ax[plot_pos].plot(angles, value, 'o-', linewidth=1, color=getColor(label))
        ax[plot_pos].fill(angles, value, color=getColor(label), alpha=0.25)
    for i in range(0, len(labels_kind)):
        ax[i].set_title(labels_kind[i])
        ax[i].set_theta_offset(np.pi / 2)
        ax[i].set_theta_direction(-1)
        ax[i].set_thetagrids(np.degrees(angles[:-1]), feature)
        ax[i].set_ylim(0, 10)
        ax[i].set_rlabel_position(180 / N)
        ax[i].grid(True)
        ax[i].tick_params(color='#222222')
        ax[i].tick_params(axis='y', labelsize=8)
        ax[i].grid(color='#AAAAAA')
        ax[i].spines['polar'].set_color('#222222')
        ax[i].set_facecolor('#FAFAFA')
    plt.suptitle(title)
    plt.show()

X, y = load_iris(return_X_y=True)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.5, random_state=0)
gnb = GaussianNB()
y_pred = gnb.fit(X_train, y_train).predict(X_test)
print("准确率:%.2f" % accuracy_score(y_test, y_pred))
print("精准率:%.2f" % precision_score(y_test, y_pred, average='macro'))
print("召回率:%.2f" % recall_score(y_test, y_pred, average='macro'))
print("F1:%.2f" % f1_score(y_test, y_pred, average='macro'))

#只画前10行的雷达图
radar(iris.head(10), title='Iris 雷达图')