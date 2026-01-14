import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import numpy as np
# 使用最小二乘法进行线性回归

# 1. 读取CSV数据
# 假设CSV文件名为RMUC_5_28.csv，列名分别为x1,x2,x3,x4,x5,x6,y
df = pd.read_csv("7_24_all.csv")

# 打印列名，检查实际列名
print("数据的列名：", df.columns.tolist())

# 2. 分离自变量X和因变量y
#X = df[['x1', 'x2', 'x3', 'x4', 'x5', 'x6']]  # 6个自变量
X = df[['I1','I2', 'I3','I4','I5','I6']]
#X = df[['I1','I2', 'I3']]
y = df['I0']  # 因变量

# 3. 拟合线性回归模型
model = LinearRegression()  # 初始化模型（默认使用最小二乘法）
model.fit(X, y)  # 拟合数据

# 4. 输出参数
#print("截距b：", model.intercept_)
#print("系数a1到a6：", model.coef_)  # 顺序对应x1到x6

print(f"截距b：{model.intercept_:.5f}")  # 显示10位小数，可以根据需要调整
print("系数a1到a6：", [f"{coef:.5f}" for coef in model.coef_])  # 对每个系数显示10位小数

# 5. 评估拟合效果
y_pred = model.predict(X)  # 预测值
r2 = r2_score(y, y_pred)  # R²值
print(f"R²值：{r2:.4f}（越接近1越好）")
