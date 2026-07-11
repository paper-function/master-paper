# 协同制导_落角约束_20260711

简介：弹道角差距小，约 1°左右。

本版本为多飞行器协同制导与落角约束 Simulink 版本。制导律保持全程连续融合，不采用分阶段切换：

```text
u = w_fuzzy * u_coop + (1 - w_fuzzy) * u_png
```

其中 PNG 分量包含标称落角约束和 ZEM 预测脱靶修正，用于满足命中精度要求；未使用强制目标点偏置。

## 最终指标

```text
最大脱靶量: 9.853 m      <= 10 m
到达时间差: 0.129 s      <= 5 s
最大落角误差: 1.008 deg  <= 3 deg
```

## 单机结果

```text
1号 miss=8.804 m, t=13.598 s, theta=-45.096 deg
2号 miss=9.651 m, t=13.567 s, theta=-44.300 deg
3号 miss=2.825 m, t=13.534 s, theta=-44.882 deg
4号 miss=9.853 m, t=13.502 s, theta=-45.023 deg
5号 miss=8.215 m, t=13.469 s, theta=-46.008 deg
```

## 文件说明

- `Ctrl_For_HRV_20260625.slx`: 当前达标 Simulink 模型。
- `updateCooperativeGuidanceCentralBlock.m`: 写入中心协同模糊制导模块的脚本。
- `HRV_Model.m`: HRV 初始经纬高、初始姿态与动力学模型。
- `evaluateMultiVehicleCooperativeMetrics.m`: 五机脱靶量、到达时间、落角误差评估脚本。
- `plotMultiVehicleGuidanceCurves.m`: 曲线绘制脚本。
- `results_20260711_183056/`: 最终版本曲线图和数据。

