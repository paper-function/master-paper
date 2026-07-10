function n = PNG_BTT_3D_Intercept(u)

%% ===== 输入 =====
X=u(1);Y=u(2);H=u(3);
v_x=u(4);v_y=u(5);v_z=u(6);

X_t=u(7);Y_t=u(8);Z_t=u(9);
v_x_t=u(10);v_y_t=u(11);v_z_t=u(12);

Q=u(13);

%% ===== 参数 =====
Parameter;
N = 4;
Q0 = 1000;

R_to_earth = earth_r + H;
g = (earth_r / R_to_earth)^2 * g0;

%% ===== 向量 =====
r_m = [X;Y;H];
v_m = [v_x;v_y;v_z];

r_t = [X_t;Y_t;Z_t];
v_t = [v_x_t;v_y_t;v_z_t];

r = r_t - r_m;
v_rel = v_t - v_m;

R = norm(r);
V = norm(v_m);

%% ===== 数值保护 =====
if R < 1e-6 || V < 1e-6
    n = [0;0];
    return;
end

%% ===== 预测视线（关键）=====
tau = R / (V + 1e-6);
r_eff = r + tau * v_rel;

r_hat = r_eff / (norm(r_eff)+1e-6);
v_hat = v_m / V;

%% ===== 速度坐标系 =====
ex = v_hat;

h = cross(v_hat, r_hat);
h_norm = norm(h);

if h_norm < 1e-6
    ez = [0;0;1];
else
    ez = h / h_norm;
end

ey = cross(ez, ex);

%% ===== 分量误差 =====
r_y = dot(r_hat, ey);
r_z = dot(r_hat, ez);

%% ===== 分量PN =====
ay = N * V * r_y;
az = N * V * r_z;

a_cmd = ay * ey + az * ez;

%% ===== 动压调度 =====
scale = Q / (Q + Q0);
a_cmd = a_cmd * scale;

%% ===== 转过载 =====
ny_cmd = dot(ey, a_cmd) / g;
nz_cmd = dot(ez, a_cmd) / g;

%% ===== 输出 =====
n = [ny_cmd; nz_cmd];

end