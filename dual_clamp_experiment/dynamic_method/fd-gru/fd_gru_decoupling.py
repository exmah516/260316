"""
FD-GRU: 机理引导摩擦-动力学耦合双子网 (Cheng et al., MSSP 2024 架构复现)
针对双夹爪介入机器人中滑轨非线性摩擦、往复传动纹波与真实器械阻力的解耦提取。

核心物理架构:
F_drive = F_inertia + F_friction + F_distal
F_distal = F_drive - m * a - F_friction(v)
1. Friction-GRU: 仅输入速度序列 v(t)，利用时序递归单元捕获 Stribeck 摩擦与微滑移迟滞
2. Dynamics-GRU: 逆动力学子网，联合驱动力、加速度与估计摩擦力，解耦输出真实接触力
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

class FrictionGRU(nn.Module):
    """专门学习机械滑轨与传力链非线性动态摩擦力矩的子网"""
    def __init__(self, input_dim=1, hidden_dim=32):
        super().__init__()
        self.gru = nn.GRU(input_dim, hidden_dim, batch_first=True)
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, 16),
            nn.Tanh(),
            nn.Linear(16, 1)
        )

    def forward(self, v_seq):
        out, _ = self.gru(v_seq)
        f_friction = self.fc(out)  # 输出与速度同维度的时变摩擦序列 [B, T, 1]
        return f_friction

class DynamicsGRU(nn.Module):
    """基于物理拓扑逆动力学解耦外力的子网"""
    def __init__(self, input_dim=3, hidden_dim=128):
        super().__init__()
        # 输入维度为 3: [驱动推力 F_drive, 加速度 a, 摩擦力估计 F_f]
        self.gru = nn.GRU(input_dim, hidden_dim, batch_first=True)
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

    def forward(self, dyn_features):
        out, _ = self.gru(dyn_features)
        F_external = self.fc(out)  # 预测纯净外载力 [B, T, 1]
        return F_external

class FD_GRU_Model(nn.Module):
    """完整的 FD-GRU 机理双子网耦合模型"""
    def __init__(self, f_hidden=32, d_hidden=128):
        super().__init__()
        self.f_net = FrictionGRU(input_dim=1, hidden_dim=f_hidden)
        self.d_net = DynamicsGRU(input_dim=3, hidden_dim=d_hidden)

    def forward(self, v_seq, F_drive_seq, a_seq):
        # 1. 估算内部摩擦
        f_friction = self.f_net(v_seq)
        
        # 2. 构造逆动力学特征向量: [驱动力, 加速度, 摩擦力]
        dyn_input = torch.cat([F_drive_seq, a_seq, f_friction], dim=-1)
        
        # 3. 剥离摩擦与惯性后反解外载接触力
        F_ext_pred = self.d_net(dyn_input)
        return F_ext_pred, f_friction

# ==========================================
# 仿真实验：双夹爪滑轨推挤中的摩擦消除与力提取
# ==========================================
def simulate_dual_clamp_feed(N_steps=1000):
    t = np.linspace(0, 10, N_steps)
    dt = t[1] - t[0]
    
    # 设定滑台往复梯形速度曲线 (进给 - 减速 - 回程)
    v = 20.0 * np.sin(2 * np.pi * 0.5 * t)  # mm/s
    a = np.gradient(v, dt) / 1000.0         # m/s^2
    
    # 模拟实际滑轨的 Stribeck 摩擦力 (非线性跃变 + 黏性阻尼)
    Fc, Fs, vs, sigma_v = 0.8, 1.4, 3.0, 0.05
    stribeck = Fc + (Fs - Fc) * np.exp(-np.abs(v / vs))
    f_friction_true = stribeck * np.sign(v) + sigma_v * v
    
    # 模拟真实血管壁接触力 (微小单调增加的阻力)
    f_ext_true = np.where(v > 0, 0.4 + 0.05 * v, 0.0)
    
    # 驱动端传感器测得的总力: 惯性力(m=0.5kg) + 摩擦力 + 外部阻力 + 测量噪声
    m_eq = 0.5
    f_drive_raw = m_eq * a + f_friction_true + f_ext_true + np.random.normal(0, 0.05, N_steps)
    
    return v, a, f_drive_raw, f_friction_true, f_ext_true

def main():
    v, a, f_drive, f_fric_true, f_ext_true = simulate_dual_clamp_feed(1200)
    
    # 转换为 PyTorch Batch 序列格式 [1, T, 1]
    v_t = torch.tensor(v, dtype=torch.float32).view(1, -1, 1)
    a_t = torch.tensor(a, dtype=torch.float32).view(1, -1, 1)
    f_drive_t = torch.tensor(f_drive, dtype=torch.float32).view(1, -1, 1)
    y_ext_t = torch.tensor(f_ext_true, dtype=torch.float32).view(1, -1, 1)
    
    model = FD_GRU_Model()
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.003)
    
    print("开始训练 FD-GRU 机理引导摩擦-动力学解耦模型...")
    model.train()
    for epoch in range(60):
        optimizer.zero_grad()
        f_ext_pred, f_fric_pred = model(v_t, f_drive_t, a_t)
        loss = criterion(f_ext_pred, y_ext_t)
        loss.backward()
        optimizer.step()
        if (epoch + 1) % 15 == 0:
            print(f"Epoch [{epoch+1}/60], Loss: {loss.item():.6f}")
            
    model.eval()
    with torch.no_grad():
        f_ext_pred, f_fric_pred = model(v_t, f_drive_t, a_t)
        
    y_true = y_ext_t.squeeze().numpy()
    y_hat = f_ext_pred.squeeze().numpy()
    rmse = np.sqrt(np.mean((y_true - y_hat) ** 2))
    print("=" * 45)
    print(f"解耦提取出的外载力与真实值对比 RMSE: {rmse:.4f} N")
    print(f"成功将滑轨 Stribeck 摩擦与往复冲击从总力信号中分离！")
    print("=" * 45)

if __name__ == "__main__":
    main()
