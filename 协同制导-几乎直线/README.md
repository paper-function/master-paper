# 协同制导-几乎直线

本版本为方案 B：每架飞行器设置各自终端落角目标，并在 PNG 分量中加入位置 ZEM 与终端速度方向约束。外层仍保持全程模糊融合协同律与 PNG，不采用分阶段切换：

```text
u = w_fuzzy * u_coop + (1 - w_fuzzy) * u_png
```

## 终端落角目标

```text
[-53 deg, -49 deg, -45 deg, -41 deg, -38 deg]
```

## 最终指标

```text
最大脱靶量: 9.587 m       <= 10 m
到达时间差: 3.814 s       <= 5 s
最大落角误差: 1.576 deg   <= 3 deg
终端倾角最小差: 3.054 deg > 2 deg
```

## 单机结果

```text
1号 miss=8.112 m, theta=-53.400 deg, theta_err=-0.400 deg
2号 miss=0.449 m, theta=-49.735 deg, theta_err=-0.735 deg
3号 miss=2.573 m, theta=-45.629 deg, theta_err=-0.629 deg
4号 miss=9.587 m, theta=-42.576 deg, theta_err=-1.576 deg
5号 miss=8.529 m, theta=-38.407 deg, theta_err=-0.407 deg
```

## 文件说明

- `Ctrl_For_HRV_20260625.slx`: 当前 Simulink 模型。
- `updateCooperativeGuidanceCentralBlock.m`: 中心协同模糊制导模块生成脚本。
- `HRV_Model.m`: HRV 初始经纬高、初始姿态与动力学模型。
- `evaluateMultiVehicleCooperativeMetrics.m`: 脱靶量、到达时间、落角误差、终端倾角差评估脚本。
- `plotMultiVehicleGuidanceCurves.m`: 曲线绘制脚本。
- `results_20260711_233431/`: 姿态角、过载、打击路径、弹道倾角、弹道偏角等曲线图和数据。

