function [sys,x0,str,ts,simStateCompliance] = HRV_Model(t,x,u,flag)

switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);  
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 14; %经度，纬度，x，y，H，V，航迹方位角，航迹倾斜角，迎角，侧滑角，倾侧角，滚转角速度，俯仰角速度，偏航角速度
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 14 + 12 + 14 + 2;
sizes.NumInputs      = 9; %右升降副翼delta_a，左升降副翼delta_e，方向舵delta_r，摄动系数xishu
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1; % at least one sample time is needed
sys = simsizes(sizes);

Parameter; %基本参数文件
global b_alpha b_beta b_miu phi_alpha phi_beta phi_miu b_p b_q b_r phi_p phi_q phi_r
b_alpha=0; b_beta=0; b_miu=0; phi_alpha=0; phi_beta=0; phi_miu=0;
b_p=0; b_q=0; b_r=0; phi_p=0; phi_q=0; phi_r=0;
global Q Ma
global dot_lon dot_lat dot_X dot_Y dot_H dot_V dot_theta dot_psi dot_alpha dot_beta dot_miu dot_p dot_q dot_r
dot_lon = 0; dot_lat = 0; dot_X = 0; dot_Y = 0; dot_H = 0; dot_V = 0; dot_theta = 0; dot_psi = 0; 
dot_alpha = 0; dot_beta = 0; dot_miu = 0; dot_p = 0; dot_q = 0; dot_r = 0;
global D Y1;

H0 = 55690; %初始高度，单位m
V0 = 5000; %初始速度，单位m/s
[Ma,Q] = M_Q(H0,V0);
x0  = [0,0,0,0,H0,V0,16.7*pi/180,-29.48*pi/180,0*pi/180,0*pi/180,0*pi/180,0*pi/180,0*pi/180,0*pi/180]; %经度，纬度，x，y，H，V，航迹方位角，航迹倾斜角，迎角，侧滑角，倾侧角，滚转角速度，俯仰角速度，偏航角速度

str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
global b_alpha b_beta b_miu phi_alpha phi_beta phi_miu b_p b_q b_r phi_p phi_q phi_r
global Q Ma
global dot_lon dot_lat dot_X dot_Y dot_H dot_V dot_theta dot_psi dot_alpha dot_beta dot_miu dot_p dot_q dot_r
global D Y1;
Parameter;
%状态向量，各分量依次为 经度，纬度，X，Y，H，V，航迹偏角，航迹倾角，迎角，侧滑角，倾侧角，滚转角速度，俯仰角速度，偏航角速度
lon=x(1); lat=x(2); X=x(3); Y=x(4); H=x(5); V=x(6); psi=x(7); theta=x(8); alpha=x(9); beta=x(10); miu=x(11); p=x(12); q=x(13); r=x(14);
%输入，各分量依次�?右升降副翼delta_a，左升降副翼delta_e，方向舵delta_r，摄动系数xishu
delta_a=u(1); delta_e=u(2); delta_r=u(3); C_L_d=u(4);C_D_d=u(5);C_Y_d=u(6);C_l_d=u(7);C_m_d=u(8);C_n_d=u(9);

[Ma,Q] = M_Q(H,V);
%计算气动系数
[C_L_alpha,C_L_delta_a,C_L_delta_e] = Coefficient_Lift(alpha,Ma,delta_a,delta_e,delta_r);
[C_D_alpha,C_D_delta_a,C_D_delta_e,C_D_delta_r] = Coefficient_Drag(alpha,Ma,delta_a,delta_e,delta_r);
[C_Y_beta,C_Y_delta_a,C_Y_delta_e,C_Y_delta_r] = Coefficient_Side(alpha,Ma,delta_a,delta_e,delta_r);
% [C_L_alpha,C_L_delta_a,C_L_delta_e] = Coefficient_Lift(alpha,Ma,0,0,0);
% [C_D_alpha,C_D_delta_a,C_D_delta_e,C_D_delta_r] = Coefficient_Drag(alpha,Ma,0,0,0);
% [C_Y_beta,C_Y_delta_a,C_Y_delta_e,C_Y_delta_r] = Coefficient_Side(alpha,Ma,0,0,0);
[C_l_beta,C_l_delta_a,C_l_delta_e,C_l_delta_r,C_l_r,C_l_p] = Coefficient_RollingMoment(alpha,Ma,delta_a,delta_e,delta_r);
[C_m_alpha,C_m_delta_a,C_m_delta_e,C_m_delta_r,C_m_q] = Coefficient_PitchMoment(alpha,Ma,delta_a,delta_e,delta_r);
[C_n_beta,C_n_delta_a,C_n_delta_e,C_n_delta_r,C_n_p,C_n_r] = Coefficient_YawingMoment(alpha,Ma,delta_a,delta_e,delta_r);
C_L = C_L_alpha+C_L_delta_a+C_L_delta_e;
C_D = C_D_alpha+C_D_delta_a+C_D_delta_e+C_D_delta_r;
C_Y = C_Y_beta*beta+C_Y_delta_a+C_Y_delta_e+C_Y_delta_r;
C_l = C_l_beta*beta+C_l_delta_a*delta_a+C_l_delta_e*delta_e+C_l_delta_r*delta_r+C_l_r*(r*b/(2*V))+C_l_p*(p*b/(2*V));
C_m = C_m_alpha+C_m_delta_a*delta_a+C_m_delta_e*delta_e+C_m_delta_r*delta_r+C_m_q*(q*c/(2*V));
C_n = C_n_beta*beta+C_n_delta_a*delta_a+C_n_delta_e*delta_e+C_n_delta_r*delta_r+C_n_p*(p*b/(2*V))+C_n_r*(r*b/(2*V));

%摄动�?
Delta_C_L = C_L_d*C_L; Delta_C_D = C_D_d*C_D; Delta_C_Y = C_Y_d*C_Y;
Delta_C_l = C_l_d*C_l; Delta_C_m = C_m_d*C_m; Delta_C_n = C_n_d*C_n;

D = Q*S*(C_D+Delta_C_D); %阻力
L = Q*S*(C_L+Delta_C_L); %升力
Y1 = Q*S*(C_Y+Delta_C_Y); %侧力
l_A = Q*S*b*(C_l+Delta_C_l); %滚转力矩
m_A = Q*S*c_A*(C_m+Delta_C_m); %俯仰力矩
n_A = Q*S*b*(C_n+Delta_C_n); %偏航力矩

R = earth_r+H; %R为地心距
g = (earth_r/R)^2*g0;
%外扰
d_qiangdu = 0;
d_alpha = d_qiangdu*0.005*sin(0.2*pi*t); d_beta = d_qiangdu*0.005*sin(0.2*pi*t); d_miu = d_qiangdu*0.005*sin(0.2*pi*t);
d_p = d_qiangdu*0.007*sin(pi*t); d_q = d_qiangdu*0.01*sin(pi*t); d_r = 0;
%14个微分方�?
dot_lon = V*cos(theta)*sin(psi)/(R*cos(lat));
dot_lat = V*cos(theta)*cos(psi)/R;
dot_X = V*cos(theta)*cos(psi);
dot_Y = V*cos(theta)*sin(psi);
dot_H = V*sin(theta);
dot_V = -D/m-g*sin(theta)+w_e^2*R*cos(lat)*(sin(theta)*cos(lat)-cos(theta)*sin(lat)*cos(psi));
% dot_V = 0;
dot_theta = (L*cos(miu)-Y1*sin(miu)-m*g*cos(theta))/(m*V)+V*cos(theta)/R+2*w_e*cos(lat)*sin(psi)+w_e^2*R*cos(lat)*(cos(theta)*cos(lat)+sin(theta)*cos(psi)*sin(lat))/V;
% w_e=0;
dot_psi = (Y1*cos(miu)+L*sin(miu))/(m*V*cos(theta))+V*cos(theta)*sin(psi)*tan(lat)/R+2*w_e*(sin(lat)-cos(lat)*cos(psi)*tan(theta))+w_e^2*R*cos(lat)*sin(lat)*sin(psi)/(V*cos(theta));
dot_alpha = -p*cos(alpha)*tan(beta)+q-r*sin(alpha)*tan(beta)+(m*g*cos(theta)*cos(miu)-L)/(m*V*cos(beta))+d_alpha;
dot_beta = p*sin(alpha)-r*cos(alpha)+(m*g*cos(theta)*sin(miu)+Y1)/(m*V)+d_beta;
dot_miu = p*cos(alpha)*sec(beta)+r*sin(alpha)*sec(beta)+(-m*g*cos(miu)*cos(theta)*tan(beta)+L*(sin(miu)*tan(theta)+tan(beta))+Y1*cos(miu)*tan(theta)*cos(beta))/(m*V)+d_miu;
dot_p = l_A/I_xx+(I_yy-I_zz)*q*r/I_xx+d_p;
dot_q = m_A/I_yy+(I_zz-I_xx)*p*r/I_yy+d_q;
dot_r = n_A/I_zz+(I_xx-I_yy)*p*q/I_zz+d_r;

%姿态角和角速率写为仿射非线性形�?
f_s = [(m*g*cos(theta)*cos(miu)-Q*S*C_L)/(m*V*cos(beta));
       (m*g*cos(theta)*sin(miu)+Q*S*C_Y)/(m*V);
       (-m*g*cos(theta)*tan(beta)*cos(miu)+Q*S*C_L*(tan(theta)*sin(miu)+tan(beta) ...
       )+Q*S*C_Y*tan(theta)*cos(beta)*cos(miu))/(m*V)];
g_s = [-cos(alpha)*tan(beta) 1 -sin(alpha)*tan(beta);
       sin(alpha)            0 -cos(alpha);
       cos(alpha)*sec(beta)  0 sin(alpha)*sec(beta)];
phi_s = [(-Q*S*Delta_C_L)/(m*V*cos(beta))+d_alpha;
         (Q*S*Delta_C_Y)/(m*V)+d_beta;
         (Q*S*Delta_C_L*(tan(theta)*sin(miu)+tan(beta))+Q*S*Delta_C_Y*tan(theta)*cos(beta)*cos(miu))/(m*V)+d_miu];
w = [p;q;r];
Dot_flowangle = f_s+g_s*w;
b_alpha = Dot_flowangle(1); b_beta = Dot_flowangle(2); b_miu = Dot_flowangle(3); %模型辅助�?phi_alpha = phi_s(1); phi_beta = phi_s(2); phi_miu = phi_s(3); %模型不确定性与干扰

f_f = [(I_yy-I_zz)*q*r/I_xx+Q*S*b*(C_l_beta*beta+C_l_p*(p*b/(2*V))+C_l_r*(r*b/(2*V)))/I_xx;
       (I_zz-I_xx)*p*r/I_yy+Q*S*c_A*(C_m_alpha+C_m_q*(q*c_A/(2*V)))/I_yy;
       (I_xx-I_yy)*p*q/I_zz+Q*S*b*(C_n_beta*beta+C_n_p*(p*b/(2*V))+C_n_r*(r*b/(2*V)))/I_zz];
g_f = [Q*S*b*C_l_delta_a/I_xx   Q*S*b*C_l_delta_e/I_xx   Q*S*b*C_l_delta_r/I_xx;
       Q*S*c_A*C_m_delta_a/I_yy Q*S*c_A*C_m_delta_e/I_yy Q*S*c_A*C_m_delta_r/I_yy;
       Q*S*b*C_n_delta_a/I_zz   Q*S*b*C_n_delta_e/I_zz   Q*S*b*C_n_delta_r/I_zz];
phi_f = [Q*S*b*Delta_C_l/I_xx+d_p;
         Q*S*c_A*Delta_C_m/I_yy+d_q;
         Q*S*b*Delta_C_n/I_zz+d_r];
u = [delta_a;delta_e;delta_r];
Dot_anglerate = f_f+g_f*u;
b_p = Dot_anglerate(1); b_q = Dot_anglerate(2); b_r = Dot_anglerate(3); %模型辅助�?phi_p = phi_f(1); phi_q = phi_f(2); phi_r = phi_f(3); %模型不确定性和外扰

x_dot=[dot_lon,dot_lat,dot_X,dot_Y,dot_H,dot_V,dot_psi,dot_theta,dot_alpha,dot_beta,dot_miu,dot_p,dot_q,dot_r];
sys = x_dot;

function sys=mdlUpdate(t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u)
global b_alpha b_beta b_miu phi_alpha phi_beta phi_miu b_p b_q b_r phi_p phi_q phi_r
global Q Ma
global dot_lon dot_lat dot_X dot_Y dot_H dot_V dot_theta dot_psi dot_alpha dot_beta dot_miu dot_p dot_q dot_r
global D Y1;
Parameter;
lon=x(1); lat=x(2); X=x(3); Y=x(4); H=x(5); V=x(6); psi=x(7); theta=x(8); alpha=x(9); beta=x(10); miu=x(11); p=x(12); q=x(13); r=x(14);
out = [lon,lat,X,Y,H,V,psi,theta,alpha,beta,miu,p,q,r,...
       b_alpha,b_beta,b_miu,phi_alpha,phi_beta,phi_miu,b_p,b_q,b_r,phi_p,phi_q,phi_r,...
       Q,Ma,...
       dot_lon,dot_lat,dot_X,dot_Y,dot_H,dot_V,dot_theta,dot_psi,dot_alpha,dot_beta,dot_miu,dot_p,dot_q,dot_r];
sys =out;


function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];


