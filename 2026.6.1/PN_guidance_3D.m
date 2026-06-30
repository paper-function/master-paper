function n = PN_guidance_3D(u)
%%读取输入
X=u(1);Y=u(2);H=u(3);v_x=u(4);v_y=u(5);v_z=u(6);
X_target=u(7);Y_target=u(8);Z_target=u(9);v_x_target=u(10);v_y_target=u(11);v_z_target=u(12);
%%参数设置
Parameter; %基本参数文件
N = 4;
R_to_earth = earth_r+H; %R_to_earth为地心距
g = (earth_r/R_to_earth)^2*g0;
%%
r_m=[X;Y;H];
v_m=[v_x;v_y;v_z];
r_t=[X_target;Y_target;Z_target];
v_t=[v_x_target;v_y_target;v_z_target];
R = r_t - r_m;
V_r = v_t - v_m;

R_norm = norm(R);

omega_LOS = cross(R, V_r) / (R_norm^2);
a_cmd = N * cross(V_r, omega_LOS);

% ===== 速度坐标系 =====
e_v = v_m / norm(v_m);
e_z = [0;0;1];

e_y = cross(e_z, e_v);
e_y = e_y / norm(e_y);

e_zv = cross(e_v, e_y);

% % =========================================================
% % ===== 鲁棒速度坐标系构造（核心改进） =====
% % =========================================================
% 
% % 速度方向
% e_v = v_m / norm(v_m);
% 
% % 自动选择参考轴（避免平行）
% if abs(e_v(3)) < 0.9
%     ref = [0;0;1];
% else
%     ref = [1;0;0];
% end
% 
% % 侧向方向
% e_y = cross(ref, e_v);
% e_y = e_y / norm(e_y);
% 
% % 法向方向
% e_zv = cross(e_v, e_y);
% e_zv = e_zv / norm(e_zv);

% ===== 投影 =====
a_n = dot(a_cmd, e_zv);
a_l = dot(a_cmd, e_y);

ny_cmd = a_l / g;
nz_cmd = a_n / g;

n=[ny_cmd;nz_cmd];
% miu_cmd = atan2(ny_cmd, nz_cmd);

end