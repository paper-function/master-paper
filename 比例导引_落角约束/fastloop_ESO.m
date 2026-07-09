function [sys,x0,str,ts,simStateCompliance] = fastloop_ESO(t,x,u,flag)

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

sizes.NumContStates  = 6; %p_hat q_hat r_hat bp_hat bq_hat br_hat
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6; %phat qhat rhat dphat dqhat drhat
sizes.NumInputs      = 6; %p q r bp bq br
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;  % at least one sample time is needed
sys = simsizes(sizes);

x0  = [0,0,0,0,0,0];
str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
bandwudth_f = 10;
beta_f_1 = bandwudth_f*2; beta_f_2 = bandwudth_f^2;

p_hat = x(1); 
q_hat = x(2); 
r_hat = x(3); 
d_p_hat = x(4); 
d_q_hat = x(5); 
d_r_hat = x(6);
p = u(1);
q = u(2);
r = u(3);
% p = u(1) + 0.003 * sin(2*pi/2 * t);
% q = u(2) + 1*10e-8 * sin(2*pi/2 * t);
% r = u(3) + 0.005 * sin(2*pi/2 * t);

b_p = u(4); b_q = u(5); b_r = u(6);

p_hat_dot = b_p + d_p_hat + beta_f_1*(p - p_hat);
q_hat_dot = b_q + d_q_hat + beta_f_1*(q - q_hat);
r_hat_dot = b_r + d_r_hat + beta_f_1*(r - r_hat);

d_p_hat_dot = beta_f_2*(p - p_hat);
d_q_hat_dot = beta_f_2*(q - q_hat);
d_r_hat_dot = beta_f_2*(r - r_hat);

x_dot=[p_hat_dot,q_hat_dot,r_hat_dot,d_p_hat_dot,d_q_hat_dot,d_r_hat_dot];
sys = x_dot;

function sys=mdlUpdate(t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u)
p_hat = x(1); q_hat = x(2); r_hat = x(3); d_p_hat = x(4); d_q_hat = x(5); d_r_hat = x(6);
out = [p_hat,q_hat,r_hat,d_p_hat,d_q_hat,d_r_hat];
sys = out;


function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;    
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];


