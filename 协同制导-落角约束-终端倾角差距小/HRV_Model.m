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

sizes.NumContStates  = 14; %缁忓害锛岀含搴︼紝x锛寉锛孒锛孷锛岃埅杩规柟浣嶈锛岃埅杩瑰炬枩瑙掞紝杩庤锛屼晶婊戣锛屽句晶瑙掞紝婊氳浆瑙掗熷害锛屼刊浠拌閫熷害锛屽亸鑸閫熷害
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 14 + 12 + 14 + 2;
sizes.NumInputs      = 9; %鍙冲崌闄嶅壇缈糳elta_a锛屽乏鍗囬檷鍓考delta_e锛屾柟鍚戣埖delta_r锛屾憚鍔ㄧ郴鏁皒ishu
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1; % at least one sample time is needed
sys = simsizes(sizes);

Parameter; %鍩烘湰鍙傛暟鏂囦欢
global b_alpha b_beta b_miu phi_alpha phi_beta phi_miu b_p b_q b_r phi_p phi_q phi_r
b_alpha=0; b_beta=0; b_miu=0; phi_alpha=0; phi_beta=0; phi_miu=0;
b_p=0; b_q=0; b_r=0; phi_p=0; phi_q=0; phi_r=0;
global Q Ma
global dot_lon dot_lat dot_X dot_Y dot_H dot_V dot_theta dot_psi dot_alpha dot_beta dot_miu dot_p dot_q dot_r
dot_lon = 0; dot_lat = 0; dot_X = 0; dot_Y = 0; dot_H = 0; dot_V = 0; dot_theta = 0; dot_psi = 0; 
dot_alpha = 0; dot_beta = 0; dot_miu = 0; dot_p = 0; dot_q = 0; dot_r = 0;
global D Y1;

H0 = 25000; %初始高度，单位m
V0 = 2800; %初始速度，单位m/s
lon0 = 0;
lat0 = 0;
X0 = 0;
Y0 = 0;
psi0 = atan2(7500, 25000);
theta0 = atan2(-H0, hypot(25000, 7500));
vehicleIndex = localVehicleIndex();
if vehicleIndex > 0
    thetaTargets = [-45 -45 -45 -45 -45] * pi/180;
    psiTargets = [17 17 17 17 17] * pi/180;
    targetX = 25000;
    targetY = 7500;
    idealRange = H0 ./ max(-tan(thetaTargets), 0.1);
    rangeScale = [1.00 1.00 1.00 1.00 1.00];
    idealRange = idealRange .* rangeScale;
    idealX = targetX - idealRange.*cos(psiTargets);
    idealY = targetY - idealRange.*sin(psiTargets);
    idealCenterX = mean(idealX);
    idealCenterY = mean(idealY);
    compressionScale = 1.000000;
    forwardShift = 0;
    clusterCenterX = idealCenterX + forwardShift;
    clusterCenterY = idealCenterY + forwardShift*targetY/targetX;
    X0 = clusterCenterX + compressionScale*(idealX(vehicleIndex) - idealCenterX);
    Y0 = clusterCenterY + compressionScale*(idealY(vehicleIndex) - idealCenterY);
    formationX = [-200 -100 0 100 200];
    formationY = [-80 -60 -18 32 80];
    X0 = X0 + formationX(vehicleIndex);
    Y0 = Y0 + formationY(vehicleIndex);
    heightOffsets = [-120 -60 0 60 120];
    H0 = H0 + heightOffsets(vehicleIndex);
    earthRadius = 6371000 + H0;
    lat0 = X0/earthRadius;
    lon0 = Y0/(earthRadius*cos(lat0));
    nominalPsi = atan2(targetY - Y0, targetX - X0);
    nominalTheta = atan2(-H0, hypot(targetX - X0, targetY - Y0));
    thetaOffsets = [0.0 0.0 0.0 0.0 0.0] * pi/180;
    psiOffsets = [0.0 0.0 -0.35 0.0 0.0] * pi/180;
    theta0 = nominalTheta + thetaOffsets(vehicleIndex);
    psi0 = nominalPsi + psiOffsets(vehicleIndex);
end
if evalin('base','exist(''HRV_INIT'',''var'')')
    HRV_INIT = evalin('base','HRV_INIT');
    if isfield(HRV_INIT,'H0'), H0 = HRV_INIT.H0; end
    if isfield(HRV_INIT,'V0'), V0 = HRV_INIT.V0; end
    if isfield(HRV_INIT,'lon0'), lon0 = HRV_INIT.lon0; end
    if isfield(HRV_INIT,'lat0'), lat0 = HRV_INIT.lat0; end
    if isfield(HRV_INIT,'X0'), X0 = HRV_INIT.X0; end
    if isfield(HRV_INIT,'Y0'), Y0 = HRV_INIT.Y0; end
    if isfield(HRV_INIT,'psi0'), psi0 = HRV_INIT.psi0; end
    if isfield(HRV_INIT,'theta0'), theta0 = HRV_INIT.theta0; end
end
[Ma,Q] = M_Q(H0,V0);
x0  = [lon0,lat0,X0,Y0,H0,V0,psi0,theta0,0*pi/180,0*pi/180,0*pi/180,0*pi/180,0*pi/180,0*pi/180]; %经度，纬度，x，y，H，V，航迹方位角，航迹倾斜角，迎角，侧滑角，倾侧角，滚转角速度，俯仰角速度，偏航角速度

str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
global b_alpha b_beta b_miu phi_alpha phi_beta phi_miu b_p b_q b_r phi_p phi_q phi_r
global Q Ma
global dot_lon dot_lat dot_X dot_Y dot_H dot_V dot_theta dot_psi dot_alpha dot_beta dot_miu dot_p dot_q dot_r
global D Y1;
Parameter;
%鐘舵佸悜閲忥紝鍚勫垎閲忎緷娆′负 缁忓害锛岀含搴︼紝X锛孻锛孒锛孷锛岃埅杩瑰亸瑙掞紝鑸抗鍊捐锛岃繋瑙掞紝渚ф粦瑙掞紝鍊句晶瑙掞紝婊氳浆瑙掗熷害锛屼刊浠拌閫熷害锛屽亸鑸閫熷害
lon=x(1); lat=x(2); X=x(3); Y=x(4); H=x(5); V=x(6); psi=x(7); theta=x(8); alpha=x(9); beta=x(10); miu=x(11); p=x(12); q=x(13); r=x(14);
%杈撳叆锛屽悇鍒嗛噺渚濇涓?鍙冲崌闄嶅壇缈糳elta_a锛屽乏鍗囬檷鍓考delta_e锛屾柟鍚戣埖delta_r锛屾憚鍔ㄧ郴鏁皒ishu
delta_a=u(1); delta_e=u(2); delta_r=u(3); C_L_d=u(4);C_D_d=u(5);C_Y_d=u(6);C_l_d=u(7);C_m_d=u(8);C_n_d=u(9);

[Ma,Q] = M_Q(H,V);
%璁＄畻姘斿姩绯绘暟
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

%鎽勫姩鍊?
Delta_C_L = C_L_d*C_L; Delta_C_D = C_D_d*C_D; Delta_C_Y = C_Y_d*C_Y;
Delta_C_l = C_l_d*C_l; Delta_C_m = C_m_d*C_m; Delta_C_n = C_n_d*C_n;

D = Q*S*(C_D+Delta_C_D); %闃诲姏
L = Q*S*(C_L+Delta_C_L); %鍗囧姏
Y1 = Q*S*(C_Y+Delta_C_Y); %渚у姏
l_A = Q*S*b*(C_l+Delta_C_l); %婊氳浆鍔涚煩
m_A = Q*S*c_A*(C_m+Delta_C_m); %淇话鍔涚煩
n_A = Q*S*b*(C_n+Delta_C_n); %鍋忚埅鍔涚煩

R = earth_r+H; %R涓哄湴蹇冭窛
g = (earth_r/R)^2*g0;
%澶栨壈
d_qiangdu = 0;
d_alpha = d_qiangdu*0.005*sin(0.2*pi*t); d_beta = d_qiangdu*0.005*sin(0.2*pi*t); d_miu = d_qiangdu*0.005*sin(0.2*pi*t);
d_p = d_qiangdu*0.007*sin(pi*t); d_q = d_qiangdu*0.01*sin(pi*t); d_r = 0;
%14涓井鍒嗘柟绋?
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

%濮挎佽鍜岃閫熺巼鍐欎负浠垮皠闈炵嚎鎬у舰寮?
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
b_alpha = Dot_flowangle(1); b_beta = Dot_flowangle(2); b_miu = Dot_flowangle(3); %妯″瀷杈呭姪椤?phi_alpha = phi_s(1); phi_beta = phi_s(2); phi_miu = phi_s(3); %妯″瀷涓嶇‘瀹氭т笌骞叉壈

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
b_p = Dot_anglerate(1); b_q = Dot_anglerate(2); b_r = Dot_anglerate(3); %妯″瀷杈呭姪椤?phi_p = phi_f(1); phi_q = phi_f(2); phi_r = phi_f(3); %妯″瀷涓嶇‘瀹氭у拰澶栨壈

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

function vehicleIndex = localVehicleIndex()
vehicleIndex = 0;
try
    blockPath = gcb;
    token = regexp(blockPath, 'Vehicle_(\d+)', 'tokens', 'once');
    if ~isempty(token)
        vehicleIndex = str2double(token{1});
    end
catch
    vehicleIndex = 0;
end
