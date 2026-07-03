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

sizes.NumContStates  = 6; %alpha_hat beta_hat miu_hat d_alpha_hat d_beta_hat d_miu_hat
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6; %alpha_hat beta_hat miu_hat d_alpha_hat d_beta_hat d_miu_hat
sizes.NumInputs      = 6; %alpha beta miu b_alpha b_beta b_miu
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;  % at least one sample time is needed
sys = simsizes(sizes);

x0  = [0,0,0,0,0,0];
str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
bandwudth_s = 10;
beta_s_1 = bandwudth_s*2; beta_s_2 = bandwudth_s^2;

alpha_hat = x(1); 
beta_hat = x(2); 
miu_hat = x(3) ; 
d_alpha_hat = x(4); 
d_beta_hat = x(5); 
d_miu_hat = x(6);

% alpha = u(1) + 0.003 * sin(2*pi/2 * t); 
% beta = u(2) + 1*10e-8 * sin(2*pi/2 * t); 
% miu = u(3) + 0.005 * sin(2*pi/2 * t); 

alpha = u(1); 
beta = u(2); 
miu = u(3); 

b_alpha = u(4); 
b_beta = u(5); 
b_miu = u(6);

alpha_hat_dot = b_alpha + d_alpha_hat + beta_s_1*(alpha - alpha_hat);
beta_hat_dot = b_beta + d_beta_hat + beta_s_1*(beta - beta_hat);
miu_hat_dot = b_miu + d_miu_hat + beta_s_1*(miu - miu_hat);

d_alpha_hat_dot = beta_s_2*(alpha - alpha_hat);
d_beta_hat_dot = beta_s_2*(beta - beta_hat);
d_miu_hat_dot = beta_s_2*(miu - miu_hat);

x_dot=[alpha_hat_dot,beta_hat_dot,miu_hat_dot,d_alpha_hat_dot,d_beta_hat_dot,d_miu_hat_dot];
sys = x_dot;

function sys=mdlUpdate(t,x,u)
sys = [];

function sys=mdlOutputs(t,x,u)
alpha_hat = x(1); beta_hat = x(2); miu_hat = x(3); d_alpha_hat = x(4); d_beta_hat = x(5); d_miu_hat = x(6);
out = [alpha_hat,beta_hat,miu_hat,d_alpha_hat,d_beta_hat,d_miu_hat];
sys = out;


function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;    
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];


