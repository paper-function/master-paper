# 协同制导-落角约束-终端倾角差距小

本版本为多飞行器协同制导与落角约束稳定版本。制导律保持全程连续融合，不采用分阶段切换：

```text
u = w_fuzzy * u_coop + (1 - w_fuzzy) * u_png
```

PNG 分量中加入 ZEM 预测脱靶修正，用于保证命中精度；未使用强制目标偏置。

## 当前指标

```text
最大脱靶量: 8.308 m     <= 10 m
到达时间差: 0.077 s     <= 5 s
最大落角误差: 0.657 deg <= 3 deg
终端倾角最小差: 0.061 deg
```

说明：该版本命中精度、到达时间协同和落角误差均达标，但终端弹道倾角之间差距较小，未达到 2 deg 以上的饱和打击角度分散要求。

## 单机结果

```text
1号 miss=3.112 m, t=13.573 s, theta=-44.547 deg
2号 miss=7.025 m, t=13.554 s, theta=-45.292 deg
3号 miss=8.308 m, t=13.535 s, theta=-45.657 deg
4号 miss=7.720 m, t=13.515 s, theta=-45.231 deg
5号 miss=6.328 m, t=13.496 s, theta=-45.510 deg
```

## 文件说明

- `Ctrl_For_HRV_20260625.slx`: 当前 Simulink 模型。
- `updateCooperativeGuidanceCentralBlock.m`: 中心协同模糊制导模块生成脚本。
- `HRV_Model.m`: HRV 初始经纬高、初始姿态与动力学模型。
- `evaluateMultiVehicleCooperativeMetrics.m`: 五机脱靶量、到达时间、落角误差和终端角差评估脚本。
- `plotMultiVehicleGuidanceCurves.m`: 曲线绘制脚本。
- `results_20260711_183056/`: 姿态角、过载、打击路径、弹道角等曲线图和数据。

