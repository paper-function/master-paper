# Ctrl_For_HRV 末段 ADRC 控制能力评估结论

## 复现文件

- 结构与依赖解析脚本：`analyze_Ctrl_For_HRV_structure.m`
- 末段场景批量仿真脚本：`run_terminal_adrc_campaign.m`
- 模型结构摘要：`terminal_adrc_results/model_structure_summary.txt`
- 批量指标：`terminal_adrc_results/terminal_metrics.csv`
- 仿真数据：`terminal_adrc_results/terminal_campaign_results.mat`
- 关键曲线：`terminal_adrc_results/figures/high_dynamic_pressure.png`

## 模型证据

- `Ctrl_For_HRV.slx` 可加载并编译，`InitFcn=Parameter;`，求解器为 `ode4`，固定步长 `0.02 s`。
- 飞行器核心为 `HRV_Model.m` Level-1 MATLAB S-function，14 个连续状态依次为：经度、纬度、X、Y、高度 H、速度 V、航迹偏角 psi、航迹倾角 theta、迎角 alpha、侧滑角 beta、倾侧角 miu、滚转/俯仰/偏航角速率 p/q/r。
- 控制结构未改动：外环 `FlowAngle_Controller.m` 将 `alpha/beta/miu` 指令转换为 `p/q/r` 指令，内环 `AngleRate_Controller.m` 将角速率指令转换为三舵面指令。
- 舵机为一阶环节，增益 50，舵面输出限幅为 `±30 deg`；舵机误差通道另有限幅 `±90 deg`。
- 气动不确定性由 `Aerodynamic Parameter perturbation` 子系统给出，`HRV_Model.m` 内还包含正弦外扰项。
- 模型完整性问题：`slowloop_ESO.m` 文件中的函数名写为 `fastloop_ESO`，三个主要 S-function 均为已弃用 Level-1 MATLAB S-function，模型中存在若干未连接的 TD/Scope/Demux 遗留端口。这些问题未阻止编译，但影响长期 V&V 可维护性。

## 末段场景建模假设

原模型没有外部参考输入接口，顶层使用 `9, 0, -15 deg` 常数作为 `alpha_d, beta_d, miu_d`。批量脚本自动创建临时模型 `Ctrl_For_HRV_terminal_harness.slx`，仅将这三个常数替换为 `From Workspace` 分段线性指令，不改变 ADRC 控制器、ESO、舵机或飞行器结构。

末段制导被等效为可执行的 `alpha/beta/miu` 快变指令；初始高度、速度、姿态偏差、角速率偏差和气动扰动状态通过 `LoadInitialState` 注入。

## 批量结果摘要

| 工况 | 初始 H/V | 指令特征 | 结果 |
|---|---:|---|---|
| `high_dynamic_pressure` | 45 km / 4200 m/s | alpha 5-14 deg，beta ±2 deg，miu -35 到 20 deg | 稳定完成 |
| `nominal_terminal` | 55.69 km / 3475 m/s | alpha 7-12 deg，beta ±1 deg，miu -25 到 10 deg | 25 s 内可运行，约 25-30 s 闭环越界 |
| `thin_air_low_q` | 65 km / 2850 m/s | 高空低动压，较快姿态/倾侧指令 | 闭环越界 |
| `initial_attitude_bias` | 50 km / 3800 m/s | 初始 alpha/beta/miu 偏差 6/3/-12 deg | 闭环越界 |
| `aggressive_guidance` | 52 km / 3900 m/s | 大幅快速倾侧反转，气动扰动放大 | 闭环越界 |
| `low_altitude_fast_descent` | 38 km / 3650 m/s | 低空大下降角、大幅倾侧指令 | 闭环越界 |

稳定完成的 `high_dynamic_pressure` 指标：

- 动压范围：`12.3-17.4 kPa`
- 马赫数范围：`12.33-12.89`
- 最大跟踪误差：alpha `7.50 deg`，beta `0.25 deg`，miu `50.32 deg`
- 末段尾窗 RMS：alpha `0.032 deg`，beta `0.007 deg`，miu `1.79 deg`
- 最大舵偏：`22.75 deg`
- 舵面饱和占比：`0`

解释：该工况最终能收敛，alpha/beta 尾段精度很好，舵面未触及 `±30 deg`。但倾侧角通道在快速大幅指令阶段出现很大的瞬态误差，说明“最终收敛”不能等同于“满足末段快速机动需求”。

## 稳定工况范围

基于当前批量仿真证据，较可信的稳定范围很窄：

- 高度约 `45 km`
- 速度约 `4200 m/s`
- 动压约 `12-17 kPa`
- 马赫数约 `12.3-12.9`
- alpha 指令约 `5-14 deg`
- beta 指令约 `±2 deg`
- 倾侧角允许存在较大瞬态误差，尾段可回到约 `2 deg RMS` 以内
- 舵面峰值低于约 `23 deg`，未饱和

## 失稳或性能下降边界

- 标称高度/速度 `55.69 km / 3475 m/s` 下，分段末段指令在 `25-30 s` 附近闭环越界。
- 高空低动压、初始姿态偏差、大幅倾侧反转、低空快速下降均会触发闭环越界。
- 边界搜索的所有放大系数工况均失败，说明当前控制器对快速末段制导指令和初始偏差缺少足够裕度。

这里的“失败”不解释为 `Model_atmoscoesa` 中变量 `T` 未定义；那只是高度/状态越过大气模型有效范围后产生的派生报错。控制评价上应归类为闭环发散或模型包线越界。

## 主要限制因素

1. 倾侧角通道瞬态能力不足。稳定工况中 `miu` 最大误差仍达到约 `50 deg`，这是最强证据。
2. 控制律依赖 `g_f` / `g_s` 逆或伪逆，未见对控制效能退化、条件数、动压/Mach 变化进行调度或保护。
3. 末段指令只经过 TD，缺少面向制导约束的显式幅值/速率/加速度整形。
4. 舵面限幅在执行机构后端，控制器本体未体现抗饱和或分配保护。
5. 初始偏差和低动压下 ESO/ADRC 固定带宽鲁棒性不足。

## 是否满足末段飞行需求

当前 ADRC 不宜判定为满足末段飞行需求。它具备局部包线内的收敛能力，但对末段典型要求中的快速航迹/姿态变化、初始偏差、气动不确定性和动压变化缺少稳定裕度；尤其倾侧角快速机动和越界失败是硬限制。

## 改进建议优先级

1. 增加末段制导参考整形：对 `alpha/beta/miu` 指令加入幅值、速率、二阶加速度约束，并与 TD 参数统一设计。
2. 做 ADRC/ESO 增益调度：按 `Q, Ma, H, V` 和控制效能矩阵条件数调度外环、内环、ESO 带宽。
3. 加入舵面限幅/速率限制感知的抗饱和机制或控制分配层。
4. 对 `g_f` 求逆增加条件数监测、奇异保护和降级策略。
5. 将气动扰动、外扰、初始状态、指令接口参数化为 mask 或数据字典，避免通过脚本强行注入。
6. 迁移 Level-1 S-function，清理未连接遗留模块，修复 `slowloop_ESO.m` 函数名不一致问题。
