function y = AngleRate_Controller(H,V,alpha,beta,miu,delta_a,delta_e,delta_r,p,q,r,phi_p_hat,phi_q_hat,phi_r_hat,p_c,p_c_dot,q_c,q_c_dot,r_c,r_c_dot,alpha_c,beta_c,miu_c)

Parameter;
[Ma,Q] = M_Q(H,V);
[C_l_beta,C_l_delta_a,C_l_delta_e,C_l_delta_r,C_l_r,C_l_p] = Coefficient_RollingMoment(alpha,Ma,delta_a,delta_e,delta_r);
[C_m_alpha,C_m_delta_a,C_m_delta_e,C_m_delta_r,C_m_q] = Coefficient_PitchMoment(alpha,Ma,delta_a,delta_e,delta_r);
[C_n_beta,C_n_delta_a,C_n_delta_e,C_n_delta_r,C_n_p,C_n_r] = Coefficient_YawingMoment(alpha,Ma,delta_a,delta_e,delta_r);

f_f = [(I_yy-I_zz)*q*r/I_xx+Q*S*b*(C_l_beta*beta+C_l_p*(p*b/(2*V))+C_l_r*(r*b/(2*V)))/I_xx;
       (I_zz-I_xx)*p*r/I_yy+Q*S*c_A*(C_m_alpha+C_m_q*(q*c_A/(2*V)))/I_yy;
       (I_xx-I_yy)*p*q/I_zz+Q*S*b*(C_n_beta*beta+C_n_p*(p*b/(2*V))+C_n_r*(r*b/(2*V)))/I_zz];
g_f = [Q*S*b*C_l_delta_a/I_xx   Q*S*b*C_l_delta_e/I_xx   Q*S*b*C_l_delta_r/I_xx;
       Q*S*c_A*C_m_delta_a/I_yy Q*S*c_A*C_m_delta_e/I_yy Q*S*c_A*C_m_delta_r/I_yy;
       Q*S*b*C_n_delta_a/I_zz   Q*S*b*C_n_delta_e/I_zz   Q*S*b*C_n_delta_r/I_zz];
phi_f_hat = [phi_p_hat;phi_q_hat;phi_r_hat];
w_c_dot = [p_c_dot;q_c_dot;r_c_dot];
e_f = [p-p_c;q-q_c;r-r_c];
g_s = [-cos(alpha)*tan(beta) 1 -sin(alpha)*tan(beta);
       sin(alpha)            0 -cos(alpha);
       cos(alpha)*sec(beta)  0 sin(alpha)*sec(beta)];
e_s = [alpha-alpha_c;beta-beta_c;miu-miu_c];
%控制律表达式
if rank(g_f)==3
    g_f_inv = inv(g_f);
else
    g_f_inv = pinv(g_f);
end
K1 = diag([10 10 10]); %[4 6 4] [10 10 10]

% CCI_AngleRate = (tanh(e_f'*f_f)+1)/2;
CCI_AngleRate = CCI(e_f,f_f);
CCI_AngleRate_value = e_f'*f_f;
Delta_d = g_f_inv*(-CCI_AngleRate*f_f-phi_f_hat+w_c_dot-K1*e_f-g_s'*e_s);
% Delta_d = g_f_inv*(-f_f-phi_f_hat+w_c_dot-K1*e_f-g_s'*e_s);
y = [Delta_d;CCI_AngleRate];
end