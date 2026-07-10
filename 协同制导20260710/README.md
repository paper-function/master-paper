# 协同制导20260710

本版本依据 Zhao et al. (2025) *Cooperative guidance under multiple constraints without active speed control* 的第 3.2 节实现协同制导与比例导引（PNG）的模糊线性融合。

## 主要文件

- `cooperativeFuzzyGuidance3D.m`: 编队级模糊融合制导核心。
- `cooperativePnOverloadCommand3D.m`: 将第 `i` 架飞行器的融合加速度命令转换为法向和侧向过载、攻角与倾侧角命令。
- `runCooperativeGuidanceDemo.m`: 点质量编队仿真示例。
- `Ctrl_For_HRV_20260625.slx`: 当前 HRV 单机六自由度 Simulink 模型。
- `tests/cooperativeGuidanceTest.m`: 连续模糊融合的自动化测试。

## 论文对应关系

当编队平均距离 `Dbar > Ra` 时，所有飞行器使用同一组由协同状态 `Coo` 和平均距离 `Dbar` 决定的模糊权重：

`a_cmd = w_coop * a_coop + (1 - w_coop) * a_PNG`

当 `Dbar <= Ra` 时，`w_coop = 0`，制导律退化为纯 PNG。这里的 `Ra` 是论文规定的终端切换条件，不是任意的中途交接距离。

## 验证

在 MATLAB 当前目录下执行：

```matlab
results = runtests("tests/cooperativeGuidanceTest.m");
assert(all([results.Passed]));
```

已验证的内容包括有限命令、无轴向加速度、编队共享模糊权重，以及进入 `Ra` 后的纯 PNG 行为。

## Simulink 集成说明

当前 `Ctrl_For_HRV_20260625.slx` 是单机模型，其控制器接口只具备本机状态。连续协同融合需要每个仿真步同时提供全编队位置、速度和通信拓扑；因此应以 `cooperativePnOverloadCommand3D.m` 作为五机耦合模型的控制器接口。

`runSimulinkCooperativeImpactGuidance.m` 仅保留为旧的单机 HRV 末段诊断脚本，不是论文意义上的连续五机协同仿真。
