function [alpha_cmd, sigma_cmd] = png_guidance_3d(u)
%% 参数定义
% 导航比 (通常取 3-5)
N_long = 4.0; % 纵向导航比
N_lat  = 4.0; % 侧向导航比

% 飞行器气动参数 (简化模型)

% C_L_alpha = 2.5;    % 升力系数斜率 (1/rad)
% g     = 9.81;       % 重力加速度



% 大气参数 (简化)
% rho = 1.225;        % 空气密度 kg/m^3

%% 1. 状态提取
% 飞行器状态 [x, y, z, V, theta, psi]
% x, y, z: 位置 (NED坐标系: x北, y东, z下)
% V: 速度大小
% theta: 航迹倾角 (Flight Path Angle), 向下为正
% psi: 航向角 (Heading Angle), 顺时针为正
H_attack = u(1); Ma_attack =u(2); Q_attack = u(3); 
x_attack = u(4); y_attack = u(5); z_attack = -u(1);
V_attack = u(6); theta_attack = u(7); psi_attack = u(8);

% 目标状态 [x, y, z]
x_t = u(9);
y_t = u(10);
z_t = -u(11);
H = u(11);

R_core = earth_r+H; %R为地心距
g = (earth_r/R_core)^2*g0;
%% 2. 计算相对运动参数
% 相对位置
R_x = x_t - x_attack;
R_y = y_t - y_attack;
R_z = z_t - z_attack; % 注意：在NED系中，z向下为正，高度差需小心处理

% 视线距离 (Range)
R = sqrt(R_x^2 + R_y^2 + R_z^2);

% 视线角 (Line of Sight Angles)
% 视线倾角 (lambda_d): 在垂直平面内，相对于水平面的角度
lambda_d = atan2(-R_z, sqrt(R_x^2 + R_y^2)); 
% 视线方位角 (lambda_t): 在水平平面内，相对于北向的角度
lambda_t = atan2(R_y, R_x);

% 视线角速度 (Line of Sight Rates)
% 近似计算 (假设目标静止或低速)
% d(lambda_d)/dt
lambda_d_dot = (V_attack * sin(theta_attack) + V_attack * cos(theta_attack) * tan(lambda_d) * cos(psi_attack - lambda_t)) / R; % 简化推导，实际应使用矢量叉乘更精确
% 更通用的矢量法计算视线角速度: omega = (R x V) / |R|^2
V_vec = [V_attack*cos(theta_attack)*cos(psi_attack), V_attack*cos(theta_attack)*sin(psi_attack), -V_attack*sin(theta_attack)]; % NED速度矢量
R_vec = [R_x, R_y, R_z];
omega_vec = cross(R_vec, V_vec) / (R^2);
% 提取分量
omega_d = omega_vec(2) * cos(lambda_t) - omega_vec(1) * sin(lambda_t); % 垂直平面视线角速度近似
omega_t = (omega_vec(1) * cos(lambda_t) + omega_vec(2) * sin(lambda_t)) / cos(lambda_d); % 水平平面视线角速度

%% 3. 比例导引律计算 (PNG)
% 期望的法向加速度指令 (Normal Acceleration Commands)
% a_n = N * V * lambda_dot

% 纵向加速度指令 (垂直于速度矢量，向上为正)
% 注意符号：在NED系中，theta向下为正，需仔细处理符号
a_n_long = N_long * V_attack * omega_d; 

% 侧向加速度指令 (垂直于速度矢量，向左为正)
a_n_lat  = N_lat * V_attack * omega_t;

%% 4. 转换为控制量 (攻角 alpha 和 倾侧角 sigma)
% 总升力产生的法向过载 n_z = L / (mg)
% 动力学关系: a_n = L / m
% L = 0.5 * rho * V^2 * S * C_L
% C_L = C_L_alpha * alpha

% 首先，我们需要确定总升力 L 的大小和方向
% 纵向通道主要由升力的垂直分量负责 (L * cos(sigma))
% 侧向通道主要由升力的水平分量负责 (L * sin(sigma))

% 考虑到重力补偿 (在纵向)
% 运动方程: V * d(theta)/dt = (L*cos(sigma))/m - g*cos(theta)
% 我们希望: V * d(theta)/dt = a_n_long (导引指令)
% 所以: (L*cos(sigma))/m = a_n_long + g*cos(theta)
LoadFactor_long = (a_n_long + g*cos(theta_attack)) / g; % 包含重力补偿的纵向过载

% 侧向运动方程: V * cos(theta) * d(psi)/dt = (L*sin(sigma))/m
% 我们希望: ... = a_n_lat
LoadFactor_lat = a_n_lat / g;

% 合成总过载
n_z_total = sqrt(LoadFactor_long^2 + LoadFactor_lat^2);

% 计算倾侧角 sigma (Bank Angle)
% tan(sigma) = 侧向分量 / 纵向分量
sigma_cmd = atan2(LoadFactor_lat, LoadFactor_long);

% 计算所需的总升力 L
L_req = n_z_total * m * g;

% 计算所需的升力系数 C_L
q_bar = 0.5 * rho * V_attack^2; % 动压
if q_bar > 10 % 防止速度过低除以0
    C_L_req = L_req / (q_bar * S);
else
    C_L_req = 0;
end

% 计算攻角 alpha
alpha_cmd = C_L_req / C_L_alpha;

% 限制控制量 (饱和)
alpha_max = deg2rad(20);
sigma_max = deg2rad(80);

alpha_cmd = saturate(alpha_cmd, -alpha_max, alpha_max);
sigma_cmd = saturate(sigma_cmd, -sigma_max, sigma_max);

end

%% 辅助函数：饱和限制
function y = saturate(u, min_val, max_val)
    if u > max_val
        y = max_val;
    elseif u < min_val
        y = min_val;
    else
        y = u;
    end
end