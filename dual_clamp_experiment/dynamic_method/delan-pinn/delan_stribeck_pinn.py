"""
DeLaN-Stribeck-PINN: 深度拉格朗日动力学与 Stribeck 摩擦耦合物理网络 (Hu et al., T-IE 2025 架构复现)
针对工业机器人/紧凑型多关节机构在无独立关节扭矩传感器条件下的动力学与非线性摩擦解耦。

核心物理原理:
tau_total = M(q)*q_ddot + C(q, q_dot)*q_dot + g(q) + tau_friction(q_dot)
1. DeLaN 核心层: 网络输出下三角矩阵 L(q)，通过 M(q) = L(q)L(q)^T 保证质量矩阵对称正定与动能守恒。
2. Stribeck 摩擦层: tau_friction = (Fc + (Fs - Fc)*exp(-|v/vs|^delta))*sign(v) + Fv*v。
3. 双循环交替冻结训练策略 (Dual-loop Freezing Learning): 解耦刚体惯量与接触摩擦。
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

class DeLaN_Friction_PINN(nn.Module):
    def __init__(self, n_dof=1):
        super().__init__()
        self.n_dof = n_dof
        
        # 1. 动能矩阵网络: 参数化下三角矩阵 L(q)，确保 M(q) 严格正定 (Positive Definite)
        self.l_dim = n_dof * (n_dof + 1) // 2
        self.net_L = nn.Sequential(
            nn.Linear(n_dof, 32),
            nn.Softplus(),
            nn.Linear(32, self.l_dim)
        )
        
        # 2. 势能网络: 输出势能标量 V(q)，保守力项 g(q) = dV/dq
        self.net_V = nn.Sequential(
            nn.Linear(n_dof, 32),
            nn.Softplus(),
            nn.Linear(32, 1)
        )
        
        # 3. 显式可辨识的 Stribeck 摩擦物理参数 (可训练变量)
        self.Fc = nn.Parameter(torch.tensor([0.4], dtype=torch.float32))    # 库仑摩擦
        self.Fs = nn.Parameter(torch.tensor([0.8], dtype=torch.float32))    # 最大静摩擦
        self.vs = nn.Parameter(torch.tensor([0.05], dtype=torch.float32))   # Stribeck 临界速度
        self.Fv = nn.Parameter(torch.tensor([0.1], dtype=torch.float32))    # 黏性阻尼系数
        self.delta = 2.0  # 指数因子

    def forward(self, q, q_dot, q_ddot):
        # 构造正定惯量矩阵 M(q)
        l_vec = self.net_L(q)
        # 单自由度简化展示: M = l^2 + eps
        M = l_vec ** 2 + 1e-4
        
        # 惯性力项
        tau_inertia = M * q_ddot
        
        # Stribeck 非线性摩擦项 (平滑符号函数以保证梯度可微)
        smooth_sign = torch.tanh(q_dot / 0.005)
        stribeck_factor = self.Fc + (self.Fs - self.Fc) * torch.exp(-torch.abs(q_dot / (self.vs + 1e-5)) ** self.delta)
        tau_fric = stribeck_factor * smooth_sign + self.Fv * q_dot
        
        tau_total = tau_inertia + tau_fric
        return tau_total, tau_inertia, tau_fric

    def freeze_friction(self, freeze=True):
        """冻结摩擦参数，单独优化刚体惯量网络"""
        self.Fc.requires_grad = not freeze
        self.Fs.requires_grad = not freeze
        self.vs.requires_grad = not freeze
        self.Fv.requires_grad = not freeze

    def freeze_dynamics(self, freeze=True):
        """冻结刚体动力学网络，单独优化摩擦参数"""
        for param in self.net_L.parameters():
            param.requires_grad = not freeze
        for param in self.net_V.parameters():
            param.requires_grad = not freeze

def main():
    # 生成单自由度滑台加减速往复运动数据
    t = np.linspace(0, 5, 500)
    q = np.sin(2 * t)
    q_dot = 2 * np.cos(2 * t)
    q_ddot = -4 * np.sin(2 * t)
    
    # 真实系统：M = 0.5kg, Fc = 0.5N, Fs = 0.9N, vs = 0.08m/s, Fv = 0.15N*s/m
    tau_true = 0.5 * q_ddot + (0.5 + 0.4 * np.exp(-abs(q_dot/0.08)**2)) * np.sign(q_dot) + 0.15 * q_dot
    
    q_t = torch.tensor(q, dtype=torch.float32).view(-1, 1)
    q_dot_t = torch.tensor(q_dot, dtype=torch.float32).view(-1, 1)
    q_ddot_t = torch.tensor(q_ddot, dtype=torch.float32).view(-1, 1)
    tau_t = torch.tensor(tau_true, dtype=torch.float32).view(-1, 1)
    
    model = DeLaN_Friction_PINN(n_dof=1)
    optimizer = optim.Adam(model.parameters(), lr=0.01)
    criterion = nn.MSELoss()
    
    print("Stage 1: 联合双环粗训...")
    for epoch in range(100):
        optimizer.zero_grad()
        tau_pred, _, _ = model(q_t, q_dot_t, q_ddot_t)
        loss = criterion(tau_pred, tau_t)
        loss.backward()
        optimizer.step()
        
    print("Stage 2: 冻结刚体网络，精细辨识物理摩擦参数...")
    model.freeze_dynamics(True)
    for epoch in range(100):
        optimizer.zero_grad()
        tau_pred, _, _ = model(q_t, q_dot_t, q_ddot_t)
        loss = criterion(tau_pred, tau_t)
        loss.backward()
        optimizer.step()
        
    print("=" * 45)
    print(f"辨识出的物理参数:")
    print(f"  库仑摩擦 Fc: {model.Fc.item():.4f} (真值: 0.5000)")
    print(f"  最大静摩擦 Fs: {model.Fs.item():.4f} (真值: 0.9000)")
    print(f"  Stribeck 临界速度 vs: {model.vs.item():.4f} (真值: 0.0800)")
    print(f"  黏性阻尼 Fv: {model.Fv.item():.4f} (真值: 0.1500)")
    print("=" * 45)

if __name__ == "__main__":
    main()
