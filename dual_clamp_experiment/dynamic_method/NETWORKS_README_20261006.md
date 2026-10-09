# LSTM、GRU 与 TCN：典型结构与开源源码

整理日期：2026-10-06。以下为原作者或框架官方代码，未修改既有研究方法目录，未运行训练，也未连接实验设备。

## 源码位置

| 网络 | 本地入口 | 来源与版本 |
| --- | --- | --- |
| TCN | `tcn_official/TCN/tcn.py`，`TemporalBlock`、`TemporalConvNet` | Bai 等的论文作者仓库 `locuslab/TCN`，commit `2f8c2b817050206397458dfd1f5a25ce8a32fe65` |
| LSTM | `lstm_gru_official/rnn.py`，`LSTM`、`LSTMCell` | PyTorch 官方 `v2.8.0` 固定版本源码 |
| GRU | `lstm_gru_official/rnn.py`，`GRU`、`GRUCell` | PyTorch 官方 `v2.8.0` 固定版本源码 |
| LSTM / GRU 运行示例 | `pytorch_examples_official/word_language_model/model.py`、`main.py` | PyTorch 官方示例仓库，commit `acc295dc7b90714f1bf47f06004fc19a7fe235c4` |

两个仓库为浅克隆，保留各自 LICENSE、README 和原始代码。`rnn.py` 是供阅读的官方源码快照，依赖 PyTorch 内部模块，不能脱离 PyTorch 单独运行；实际使用通过 `torch.nn.LSTM` / `torch.nn.GRU`。其源文件 Git blob SHA 为 `1d5994f919139e0143b9d6b4a74ba088599d77a8`，LICENSE 同样来自 PyTorch v2.8.0。该版本用于固定来源，不代表最新版本。

## 1. LSTM 典型结构

```text
时间序列 X -> 单向 LSTM 层（可堆叠） -> 逐时刻 Linear -> 输出序列
                  h(t), c(t)

单元输入: x(t), h(t-1), c(t-1)
输入门 i(t)、遗忘门 f(t)、输出门 o(t)、候选记忆 g(t)
c(t) = f(t) * c(t-1) + i(t) * g(t)
h(t) = o(t) * tanh(c(t))
```

`*` 表示逐元素乘法。在线实时处理采用单向网络；双向网络会使用未来信息。

## 2. GRU 典型结构

```text
时间序列 X -> 单向 GRU 层（可堆叠） -> 逐时刻 Linear -> 输出序列
                 h(t)

单元输入: x(t), h(t-1)
重置门 r(t)、更新门 z(t)、候选状态 n(t)
h(t) = (1-z(t)) * n(t) + z(t) * h(t-1)
```

上式采用 PyTorch 的门定义。PyTorch 在候选状态中的重置门位置与最初 GRU 论文存在差异，官方源码文档明确说明了这一点；不可把所有 GRU 变体写成完全相同的公式。

## 3. TCN 典型结构

```text
X [B,Cin,T] -> 残差块(d=1) -> 残差块(d=2) -> 残差块(d=4) -> ...
           -> 任务读出层 -> 输出

单个残差块:
x -> 膨胀因果 Conv1d + WeightNorm -> ReLU -> Dropout
  -> 膨胀因果 Conv1d + WeightNorm -> ReLU -> Dropout -> (+) -> ReLU
 +----------------------------------------------------^
               恒等映射，通道数不同时用 1x1 Conv
```

作者实现用 padding 后裁掉右端样本的 `Chomp1d` 保证因果性和时间长度一致。上面的读出层由任务定义，并不在 `TemporalConvNet` 类中。对于回归任务，可以用 `Linear` 或 1x1 Conv 输出每时刻预测值，或仅取最后时刻预测。

当每块有两层、核大小 k、膨胀率为 1,2,...,2^(L-1) 时，感受野为：

```text
R = 1 + 2*(k-1)*(2^L-1)
```

PPT 第3页采用 `L=3, k=2, d=1,2,4` 的典型示意，R=15 个采样点；并非替实际机构选择的最终超参数。若采样率为 1 kHz，窗口首尾跨度仅 14 ms，不能直接假设已覆盖整个换手瞬态。最终感受野应按实际事件持续时间确定。

## 运行官方示例

先在自己的训练环境安装合适的 PyTorch。此次没有更改现有 Python 环境。

```powershell
cd pytorch_examples_official/word_language_model
python main.py --model LSTM --epochs 1 --emsize 64 --nhid 64 --nlayers 1
python main.py --model GRU --epochs 1 --emsize 64 --nhid 64 --nlayers 1
```

这是语言建模示例，用于理解官方网络调用，不是可直接训练力数据的脚本。用于测力回归时，输入换成连续传感器特征，移除词嵌入和分类 softmax，并使用连续量读出和回归损失。

TCN 的各任务启动脚本及数据依赖见 `tcn_official/README.md`。其中 `TCN/adding_problem` 提供序列回归示例。作者代码使用旧接口 `torch.nn.utils.weight_norm`，新环境可能提示弃用警告，原始源码保持不变。

## 对本任务的定义边界

- 内部“残差块”的跳跃连接，不等于物理模型输出之后的补偿残差；TCN 同样可直接预测目标力。
- 参考机构的力是否为有效标签，需要由实际受力关系决定，不能仅凭符号反向认定为真值。
- 若参考力仅在训练实验中存在，不应把参考力历史加入上线模型输入。
- 输入只使用可在线获得的当前及历史信息。物理计算量可以作为额外通道，但不是使用 TCN 的必需条件。
- 本次只整理典型结构与原版源码，不将这些源码称为已验证的夹持机构补偿算法。

## 一手资料

- TCN 论文：https://arxiv.org/abs/1803.01271
- TCN 作者仓库：https://github.com/locuslab/TCN
- LSTM 原始论文：Hochreiter & Schmidhuber, Long Short-Term Memory, Neural Computation, 1997, DOI: 10.1162/neco.1997.9.8.1735
- GRU 早期论文：Cho et al., Learning Phrase Representations using RNN Encoder-Decoder for Statistical Machine Translation, 2014：https://arxiv.org/abs/1406.1078
- 官方 LSTM 文档：https://docs.pytorch.org/docs/stable/generated/torch.nn.LSTM.html
- 官方 GRU 文档：https://docs.pytorch.org/docs/stable/generated/torch.nn.GRU.html
- 官方固定版本源码：https://github.com/pytorch/pytorch/blob/v2.8.0/torch/nn/modules/rnn.py
- 官方运行示例：https://github.com/pytorch/examples/tree/acc295dc7b90714f1bf47f06004fc19a7fe235c4/word_language_model
