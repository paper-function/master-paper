function y = FlowAngle_Controller(H,V,alpha,beta,miu,delta_a,delta_e,delta_r,theta,phi_alpha_hat,phi_beta_hat,phi_miu_hat,alpha_c,alpha_c_dot,beta_c,beta_c_dot,miu_c,miu_c_dot)

Parameter;
R = earth_r+H; %R为地心距
g = (earth_r/R)^2*g0;
[Ma,Q] = M_Q(H,V);
[C_L_alpha,C_L_delta_a,C_L_delta_e] = Coefficient_Lift(alpha,Ma,delta_a,delta_e,delta_r);
[C_Y_beta,C_Y_delta_a,C_Y_delta_e,C_Y_delta_r] = Coefficient_Side(alpha,Ma,delta_a,delta_e,delta_r);

C_L = C_L_alpha+C_L_delta_a+C_L_delta_e;
C_Y = C_Y_beta*beta+C_Y_delta_a+C_Y_delta_e+C_Y_delta_r;

f_s = [(m*g*cos(theta)*cos(miu)-Q*S*C_L)/(m*V*cos(beta));
       (m*g*cos(theta)*sin(miu)+Q*S*C_Y)/(m*V);
       (-m*g*cos(theta)*tan(beta)*cos(miu)+Q*S*C_L*(tan(theta)*sin(miu)+tan(beta))+Q*S*C_Y*tan(theta)*cos(beta)*cos(miu))/(m*V)];
g_s = [-cos(alpha)*tan(beta) 1 -sin(alpha)*tan(beta);
       sin(alpha)            0 -cos(alpha);
       cos(alpha)*sec(beta)  0 sin(alpha)*sec(beta)];
phi_s_hat = [phi_alpha_hat;phi_beta_hat;phi_miu_hat];
Omega_c_dot = [alpha_c_dot;beta_c_dot;miu_c_dot];
e_s = [alpha-alpha_c;beta-beta_c;miu-miu_c];
%控制律表达式
if rank(g_s)==3
    g_s_inv = inv(g_s);
else
    g_s_inv = pinv(g_s);
end
K_miu = 4;
if evalin('base',"exist('K_miu_override','var')")
    K_miu = evalin('base','K_miu_override');
end
K = diag([3 4 K_miu]); %[1 3 2] [3 4 4]
w_d = g_s_inv*(-f_s-phi_s_hat+Omega_c_dot-K*e_s);
y = w_d;
end
