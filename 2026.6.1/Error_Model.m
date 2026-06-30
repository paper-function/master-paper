function [sys,x0,str,ts,simStateCompliance] = Error_Model(t,x,u,flag)

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

sizes.NumContStates  = 6; %误差：迎角，侧滑角，倾侧角，w1，w2，w3
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;%姿态角误差，ew误差，ewd
sizes.NumInputs      = 33; %右升降副翼delta_a，左升降副翼delta_e，方向舵delta_r，摄动系数xishu,，高度，速度，3个姿态角指令，3个姿态角指令微分，3个姿态角指令微分微分，3个ewd，3个ewd微分
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1; % at least one sample time is needed
sys = simsizes(sizes);

Parameter; %基本参数文件
global Q Ma
global dot_error_alpha dot_error_beta dot_error_miu dot_error_w_1 dot_error_w_2 dot_error_w_3 
global e_wd_1 e_wd_2 e_wd_3
dot_error_alpha=0;dot_error_beta=0;dot_error_miu=0;dot_error_w_1=0;dot_error_w_2=0;dot_error_w_3=0;
e_wd_1=0;e_wd_2=0;e_wd_3=0;

H0 = 55690; %初始高度，单位m
V0 = 3475; %初始速度，单位m/s
[Ma,Q] = M_Q(H0,V0);
x0  = [0*pi/180;0*pi/180;0*pi/180;0*pi/180;0*pi/180;0*pi/180]; %迎角，侧滑角，倾侧角，滚转角速度，俯仰角速度，偏航角速度

str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
global Q Ma
global dot_error_alpha dot_error_beta dot_error_miu dot_error_w_1 dot_error_w_2 dot_error_w_3 
global e_wd_1 e_wd_2 e_wd_3
Parameter;

error_alpha=x(1); error_beta=x(2); error_miu=x(3); error_w_1=x(4); error_w_2=x(5); error_w_3=x(6);
e_omega = [error_alpha;error_beta;error_miu];
e_w = [error_w_1;error_w_2;error_w_3];
%输入，各分量依次为 右升降副翼delta_a，左升降副翼delta_e，方向舵delta_r，摄动系数xishu
delta_a=u(1); delta_e=u(2); delta_r=u(3); C_L_d=u(4);C_D_d=u(5);C_Y_d=u(6);C_l_d=u(7);C_m_d=u(8);C_n_d=u(9);
H=u(10);V=u(11);
alpha_c = u(12); beta_c=u(13); miu_c=u(14); dot_alpha_c=u(15); dot_beta_c=u(16); dot_miu_c=u(17); ddot_alpha_c=u(18); ddot_beta_c=u(19); ddot_miu_c=u(20);
e_wd_1 = u(21); e_wd_2 = u(22); e_wd_3 = u(23); dot_e_wd_1 = u(24); dot_e_wd_2 = u(25); dot_e_wd_3 = u(26); 
alpha=u(27);beta=u(28);miu=u(29);p=u(30);q=u(31);r=u(32);theta=u(33);


omega_c = [alpha_c;beta_c;miu_c];
dot_omega_c = [dot_alpha_c;dot_beta_c;dot_miu_c];
ddot_omega_c = [ddot_alpha_c;ddot_beta_c;ddot_miu_c];
e_wd = [e_wd_1;e_wd_2;e_wd_3];
dot_e_wd = [dot_e_wd_1;dot_e_wd_2;dot_e_wd_3];

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

%摄动值
Delta_C_L = C_L_d*C_L; Delta_C_D = C_D_d*C_D; Delta_C_Y = C_Y_d*C_Y;
Delta_C_l = C_l_d*C_l; Delta_C_m = C_m_d*C_m; Delta_C_n = C_n_d*C_n;

%%重力加速度
R = earth_r+H; %R为地心距
g = (earth_r/R)^2*g0;

%%
% 模型调整
%先得到系数
Coefficient_C_L_alpha = 1.86e-2 + 1.01e-4 + 1.01e-4;
C_L_else = C_L - Coefficient_C_L_alpha * alpha;

Coefficient_C_Y_alpha = -1.12e-7 + 1.12e-7 + 4.86e-20;
C_Y_else = C_Y - Coefficient_C_Y_alpha * alpha;

Coefficient_C_l_beta_0 = -1.402e-1;
C_l_beta_else = C_l_beta - Coefficient_C_l_beta_0;

Coefficient_C_l_p_0 = -2.99e-1;
C_l_p_else = C_l_p - Coefficient_C_l_p_0;

Coefficient_C_l_r_0 = 3.82e-1;
C_l_r_else = C_l_r - Coefficient_C_l_r_0;

Coefficient_C_m_alpha_alpha = -2.260e-3 -6.59e-5 -6.59e-5;
C_m_alpha_else = C_m_alpha - Coefficient_C_m_alpha_alpha*alpha;

Coefficient_C_m_q_0 = -1.36;
C_m_q_else = C_m_q - Coefficient_C_m_q_0;

Coefficient_C_n_p_0 = 3.68e-1;
C_n_p_else = C_n_p - Coefficient_C_n_p_0;


%将模改写为面向控制的形式
d_alpha = 0.1*0.005*sin(0.2*pi*t); d_beta = 0.1*0.005*sin(0.2*pi*t); d_miu = 0.1*0.005*sin(0.2*pi*t);
d_p = 0.1*0.007*sin(pi*t); d_q = 0.1*0.01*sin(pi*t); d_r = 0;
%姿态角环
omega = [alpha;beta;miu];
w = [p;q;r];
u = [delta_a;delta_e;delta_r];

W_oo = [-Q*S*Coefficient_C_L_alpha,0,0;
        Q*S*Coefficient_C_Y_alpha,0,0;
        Q*S*(Coefficient_C_L_alpha*(tan(theta)*sin(miu)+tan(beta))+Coefficient_C_Y_alpha*tan(theta)*cos(beta)*cos(miu))];
F_else = [-Q*S*C_L_else,0,0;
        Q*S*C_Y_else,0,0;
        Q*S*(C_L_else*(tan(theta)*sin(miu)+tan(beta))+C_Y_else*tan(theta)*cos(beta)*cos(miu))];
f_o = [(m*g*cos(theta)*cos(miu))/(m*V*cos(beta));
       (m*g*cos(theta)*sin(miu))/(m*V);
       (-m*g*cos(theta)*tan(beta)*cos(miu))/(m*V)];
g_o = [-p*cos(alpha)*tan(beta) - r*sin(alpha)*tan(beta);
       p*sin(alpha);
       r*sin(alpha)*sec(beta)];
W_ow = [0,1,0;
        0,0,-cos(alpha);
        cos(alpha)*sec(beta)];
phi_s = [(-Q*S*Delta_C_L)/(m*V*cos(beta))+d_alpha;
         (Q*S*Delta_C_Y)/(m*V)+d_beta;
         (Q*S*Delta_C_L*(tan(theta)*sin(miu)+tan(beta))+Q*S*Delta_C_Y*tan(theta)*cos(beta)*cos(miu))/(m*V)+d_miu];
D_F = f_o + F_else + phi_s;
Dot_o = W_oo*omega + W_ow*w + g_o + D_F;

%角速度环
W_wo = [0,Q*S*b*Coefficient_C_l_beta_0/I_xx,0;
        Q*S*c_A*Coefficient_C_m_alpha_alpha/I_yy,0,0;
        0,0,0];
W_ww = [(Q*S*b/I_xx)*(Coefficient_C_l_p_0*b/(2*V)),0,(Q*S*b/I_xx)*(Coefficient_C_l_r_0*b/(2*V));
        0,(Q*S*c_A/I_yy)*Coefficient_C_m_q_0*c_A/(2*V),0;
        (Q*S*b/I_zz)*Coefficient_C_n_p_0*b/(2*V),0,0];
M_else = [(Q*S*b/I_xx)*(C_l_beta_else*beta+C_l_p_else*(p*b/(2*V))+C_l_r_else*(r*b/(2*V)));
          (Q*S*c_A/I_yy)*(C_m_alpha_else+C_m_q_else*(q*c_A/(2*V)));
          (Q*S*b/I_zz)*(C_n_beta*beta+C_n_p_else*(p*b/(2*V))+C_n_r*(r*b/(2*V)))];
f_w = [(I_yy-I_zz)*q*r/I_xx;
       (I_zz-I_xx)*p*r/I_yy;
       (I_xx-I_yy)*p*q/I_zz];
g_f = [Q*S*b*C_l_delta_a/I_xx   Q*S*b*C_l_delta_e/I_xx   Q*S*b*C_l_delta_r/I_xx;
       Q*S*c_A*C_m_delta_a/I_yy Q*S*c_A*C_m_delta_e/I_yy Q*S*c_A*C_m_delta_r/I_yy;
       Q*S*b*C_n_delta_a/I_zz   Q*S*b*C_n_delta_e/I_zz   Q*S*b*C_n_delta_r/I_zz];
phi_f = [Q*S*b*Delta_C_l/I_xx+d_p;
         Q*S*c_A*Delta_C_m/I_yy+d_q;
         Q*S*b*Delta_C_n/I_zz+d_r];
D_M = M_else + phi_f;

Dot_w = W_ww*w + W_wo*omega + g_f*u + f_w + D_M;
%%
%计算误差系统
%e_omega = omega - omega_c;
%e_w = W_ow*w + W_oo*omega_c - dot_omega_c - e_wd;

A_oo = W_oo;
A_ww = W_ow*W_ww*inv(W_ow);
A_wo = W_ow*W_wo;
B = W_ow*g_f;
h = (W_ow*W_wo - W_ow*W_ww*inv(W_ow)*W_oo)*omega_c + (W_oo+W_ow*W_ww*inv(W_ow))*dot_omega_c - ddot_omega_c;
r = W_ow*W_ww*inv(W_ow)*e_wd - dot_e_wd;
D_M_error = W_ow*D_M;

dot_e_omega = A_oo*e_omega + e_w + e_wd + g_o + D_F;
dot_e_w = A_ww*e_w + A_wo*e_omega + f_w + B*u + h + r + D_M_error;


dot_error_alpha = dot_e_omega(1); dot_error_beta = dot_e_omega(2); dot_error_miu = dot_e_omega(3);
dot_error_w_1 = dot_e_w(1); dot_error_w_2 = dot_e_w(2); dot_error_w_3 = dot_e_w(3);

x_dot=[dot_error_alpha;dot_error_beta;dot_error_miu;dot_error_w_1;dot_error_w_2;dot_error_w_3];
sys = x_dot;

function sys=mdlUpdate(t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u)
global Q Ma
global dot_error_alpha dot_error_beta dot_error_miu dot_error_w_1 dot_error_w_2 dot_error_w_3 
global e_wd_1 e_wd_2 e_wd_3
Parameter;

error_alpha=x(1); error_beta=x(2); error_miu=x(3); error_w_1=x(4); error_w_2=x(5); error_w_3=x(6);

out = [error_alpha,error_beta,error_miu,error_w_1,error_w_2,error_w_3,e_wd_1,e_wd_2,e_wd_3];
sys =out;


function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];


