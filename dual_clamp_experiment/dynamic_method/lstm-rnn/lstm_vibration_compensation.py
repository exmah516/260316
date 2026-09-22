"""
LSTM-RNN 实时振动与扰动超前预测补偿器 (He et al., T-MECH 2025 架构复现)
针对血管介入机器人双夹爪切换/重夹持初期的瞬态力波动与相位滞后对冲。

核心两阶段流水线:
1. RLS-F (递归最小二乘滤波器): 在线滤除传感器高频带外白噪声与低频零漂。
2. LSTM-RNN (时序超前预测网络): 滑窗输入历史滤波信号，预测未来固定时延步长 Δt (滤波时延+执行机构响应时延) 的扰动值，
   实现零相位滞后 (Zero-phase lag) 的前馈对冲消除。
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# ==========================================
# 1. 简易 RLS 自适应滤波器 (去除零漂与高频噪声)
# ==========================================
class RLSFilter:
    def __init__(self, order=4, lmbda=0.98, delta=1.0):
        self.order = order
        self.lmbda = lmbda
        self.P = np.eye(order) * delta
        self.w = np.zeros((order, 1))

    def update(self, x, d):
        """
        x: [order, 1] 输入状态向量
        d: 期望信号/实测标量
        """
        x = x.reshape(-1, 1)
        Px = self.P @ x
        gamma = float(self.lmbda + x.T @ Px)
        k = Px / gamma
        e = d - float(self.w.T @ x)
        self.w += k * e
        self.P = (self.P - k @ x.T @ self.P) / self.lmbda
        y = float(self.w.T @ x)
        return y, e

# ==========================================
# 2. LSTM-RNN 时序超前预测网络
# ==========================================
class LSTMPredictor(nn.Module):
    def __init__(self, input_dim=1, hidden_dim=64, num_layers=2, output_dim=1):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, 32),
            nn.ReLU(),
            nn.Linear(32, output_dim)
        )

    def forward(self, x):
        # x: [Batch, Seq_len, Input_dim]
        lstm_out, _ = self.lstm(x)
        last_out = lstm_out[:, -1, :]  # 取最后一个时步隐藏状态
        pred = self.fc(last_out)
        return pred

# ==========================================
# 3. 仿真数据生成与训练验证流水线
# ==========================================
def simulate_clamp_disturbance(seq_len=2000):
    """
    模拟双夹爪往复递送过程中的力传感器信号:
    含血管恒定阻力 + 夹爪闭合冲击脉冲 + 传动链高频谐波振动 + 传感器随机噪声
    """
    t = np.linspace(0, 20, seq_len)
    dt = t[1] - t[0]
    
    # 真实远端有效力 (平缓渐增)
    F_true = 0.5 + 0.2 * np.sin(0.5 * t)
    
    # 周期性夹爪闭合冲击扰动 (每隔 2s 闭合一次，产生瞬态力峰)
    disturbance = np.zeros_like(t)
    for t_event in np.arange(2.0, 20.0, 2.0):
        idx = (t >= t_event) & (t < t_event + 0.3)
        decay = np.exp(-15 * (t[idx] - t_event))
        disturbance[idx] = 1.2 * decay * np.sin(40 * (t[idx] - t_event))
        
    # 机构电机传动振动纹波 (10Hz) + 白噪声
    ripples = 0.15 * np.sin(2 * np.pi * 10 * t)
    noise = np.random.normal(0, 0.03, seq_len)
    
    # 传感器原始测得总力
    F_raw = F_true + disturbance + ripples + noise
    return t, F_raw, disturbance, F_true

def train_and_evaluate():
    t, F_raw, d_true, F_true = simulate_clamp_disturbance(2000)
    
    # 1. 构造滑窗时序数据集 (Window size = 20, 目标为提前 step_ahead = 5 步的扰动)
    window_size = 20
    step_ahead = 5
    
    X, Y = [], []
    for i in range(len(F_raw) - window_size - step_ahead):
        X.append(F_raw[i : i + window_size])
        Y.append(d_true[i + window_size + step_ahead]) # 预测未来 step_ahead 处的扰动
        
    X = torch.tensor(np.array(X), dtype=torch.float32).unsqueeze(-1)
    Y = torch.tensor(np.array(Y), dtype=torch.float32).unsqueeze(-1)
    
    # 划分训练集与测试集
    train_size = int(0.8 * len(X))
    X_train, Y_train = X[:train_size], Y[:train_size]
    X_test, Y_test = X[train_size:], Y[train_size:]
    
    # 2. 训练 LSTM 模型
    model = LSTMPredictor(input_dim=1, hidden_dim=32, num_layers=2, output_dim=1)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.005)
    
    print("开始训练 LSTM-RNN 超前扰动预测器...")
    model.train()
    for epoch in range(50):
        optimizer.zero_grad()
        output = model(X_train)
        loss = criterion(output, Y_train)
        loss.backward()
        optimizer.step()
        if (epoch + 1) % 10 == 0:
            print(f"Epoch [{epoch+1}/50], Loss: {loss.item():.6f}")
            
    # 3. 测试与补偿消除效果
    model.eval()
    with torch.no_grad():
        d_pred = model(X_test).squeeze().numpy()
        
    y_test_np = Y_test.squeeze().numpy()
    rmse_before = np.sqrt(np.mean(y_test_np ** 2))
    rmse_after = np.sqrt(np.mean((y_test_np - d_pred) ** 2))
    suppression_rate = (1 - rmse_after / rmse_before) * 100
    
    print("=" * 45)
    print(f"原始未补偿扰动 RMSE: {rmse_before:.4f} N")
    print(f"LSTM前瞻对冲后残余 RMSE: {rmse_after:.4f} N")
    print(f"扰动消除率 (Suppression Rate): {suppression_rate:.2f}%")
    print("=" * 45)

if __name__ == "__main__":
    train_and_evaluate()
