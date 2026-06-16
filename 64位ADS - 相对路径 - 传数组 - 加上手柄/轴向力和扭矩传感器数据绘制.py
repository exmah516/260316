import os
import pandas as pd
import matplotlib.pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))

file_name = os.path.join(current_dir, 'Force_sensor_20260616_103210.csv')

# 1. 读取数据
df = pd.read_csv(file_name)

# 2. 寻找 f、g 列（即 fn_1_zero_v 和 ft_1_zero_v）首次出现非 0 数据的行
f_col = 'fn_1_zero_v'
g_col = 'ft_1_zero_v'
d_col = 'fn_1_raw_v'
e_col = 'ft_1_raw_v'

nonzero_condition = (df[f_col] != 0) | (df[g_col] != 0)
nonzero_indices = df[nonzero_condition].index

if len(nonzero_indices) == 0:
    raise ValueError("未在 f 列和 g 列中找到非 0 数据，请检查数据源！")

first_nonzero_idx = nonzero_indices[0]
print(f"检测到第一个 f、g 有非0数据的行索引为: {first_nonzero_idx}")

# 3. 截取从该行开始的所有数据
filtered_df = df.iloc[first_nonzero_idx:].copy()

# 4. 数据计算与坐标转换
filtered_df['d_minus_f'] = filtered_df[d_col] - filtered_df[f_col]
filtered_df['e_minus_g'] = filtered_df[e_col] - filtered_df[g_col]

# 时间戳 tick_ms 转换为秒（以第一个非 0 行为 0 秒起始）
start_tick = filtered_df['tick_ms'].iloc[0]
filtered_df['t_sec'] = (filtered_df['tick_ms'] - start_tick) / 1000.0

# 5. 配置科研绘图风格与中文支持
# 添加多系统常用字体，兼容 Windows ('SimHei'/'Microsoft YaHei') 和 Linux 环境
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False  # 正常显示负号
plt.rcParams['font.size'] = 12             # 全局字体大小
plt.rcParams['axes.labelsize'] = 14         # 轴标签字体大小
plt.rcParams['axes.titlesize'] = 14         # 标题字体大小
plt.rcParams['xtick.labelsize'] = 11        # X轴刻度大小
plt.rcParams['ytick.labelsize'] = 11        # Y轴刻度大小

# ---------------- 图1：d 减 f 随时间变化曲线（引入 axis1 虚线） ----------------
fig, ax1 = plt.subplots(figsize=(8, 5), dpi=300)

# 左坐标轴：绘制电压差（实线）
ln1 = ax1.plot(
    filtered_df['t_sec'], filtered_df['d_minus_f'], 
    color='#1f77b4', linewidth=1.5, label='电压差值 (d - f)'
)
ax1.set_xlabel('时间 (秒)', fontweight='bold')
ax1.set_ylabel('电压差值 (V)', fontweight='bold', color='#1f77b4')
ax1.tick_params(axis='y', labelcolor='#1f77b4')
ax1.grid(True, linestyle='--', alpha=0.4)  # 科学轻量级网格

# 右坐标轴：绘制绝对位置 axis1（虚线）
ax1_twin = ax1.twinx()
ln2 = ax1_twin.plot(
    filtered_df['t_sec'], filtered_df['axis1_pos_abs_mm'], 
    color='#d62728', linewidth=1.2, linestyle='--', label='位置 (axis1_pos)'
)
ax1_twin.set_ylabel('绝对位置 axis1 (mm)', fontweight='bold', color='#d62728')
ax1_twin.tick_params(axis='y', labelcolor='#d62728')

# 合并两轴的图例，并放置于右上角
lns1 = ln1 + ln2
labs1 = [l.get_label() for l in lns1]
ax1.legend(lns1, labs1, loc='upper right')

ax1.set_title('d列减去f列信号及axis1位置随时间变化曲线', fontweight='bold', pad=15)
plt.tight_layout()
# 保存图像时使用相对路径，与数据文件路径保持一致
plt.savefig(os.path.join(current_dir, 'd_minus_f_with_axis1.png'), bbox_inches='tight')
plt.close()


# ---------------- 图2：e 减 g 随时间变化曲线（引入 axis2 虚线） ----------------
fig, ax2 = plt.subplots(figsize=(8, 5), dpi=300)

# 左坐标轴：绘制电压差（实线）
ln3 = ax2.plot(
    filtered_df['t_sec'], filtered_df['e_minus_g'], 
    color='#ff7f0e', linewidth=1.5, label='电压差值 (e - g)'
)
ax2.set_xlabel('时间 (秒)', fontweight='bold')
ax2.set_ylabel('电压差值 (V)', fontweight='bold', color='#ff7f0e')
ax2.tick_params(axis='y', labelcolor='#ff7f0e')
ax2.grid(True, linestyle='--', alpha=0.4)

# 右坐标轴：绘制绝对位置 axis2（虚线）
ax2_twin = ax2.twinx()
ln4 = ax2_twin.plot(
    filtered_df['t_sec'], filtered_df['axis2_pos_abs_mm'], 
    color='#2ca02c', linewidth=1.2, linestyle='--', label='位置 (axis2_pos)'
)
ax2_twin.set_ylabel('绝对位置 axis2 (mm)', fontweight='bold', color='#2ca02c')
ax2_twin.tick_params(axis='y', labelcolor='#2ca02c')

# 合并两轴的图例，并放置于右上角
lns2 = ln3 + ln4
labs2 = [l.get_label() for l in lns2]
ax2.legend(lns2, labs2, loc='upper right')

ax2.set_title('e列减去g列信号及axis2位置随时间变化曲线', fontweight='bold', pad=15)
plt.tight_layout()
# 保存图像时使用相对路径，与数据文件路径保持一致
plt.savefig(os.path.join(current_dir, 'e_minus_g_with_axis2.png'), bbox_inches='tight')
plt.close()

print("图表更新完成！已成功保存为 'd_minus_f_with_axis1.png' 和 'e_minus_g_with_axis2.png'。")
