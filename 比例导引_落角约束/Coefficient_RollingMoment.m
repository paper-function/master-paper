function [C_l_beta,C_l_delta_a,C_l_delta_e,C_l_delta_r,C_l_r,C_l_p] = Coefficient_RollingMoment(alpha,Ma,delta_a,delta_e,delta_r)
%求滚转力矩系数各分量 2022.06.14 任斌
r2d = 180/pi;
% r2d = Coefficient_r2d(alpha*r2d);
alpha_deg = alpha * r2d;
C_l_beta = (-1.402e-1)+(3.326e-2)*Ma-(7.590e-4)*alpha_deg+(8.596e-6)*(alpha_deg*Ma)-(3.794e-3)*Ma^2+(2.354e-6)*alpha_deg^2-(1.044e-8)*(alpha_deg*Ma)^2+(2.219e-4)*Ma^3-(8.964e-18)*alpha_deg^3-(6.462e-6)*Ma^4+(7.419e-8)*Ma^5;
C_l_delta_a = (1.170e-4)+(2.794e-8)*(alpha*Ma)+(-1.16e-6)*delta_a+(-4.641e-11)*(alpha_deg^2*Ma^2*delta_a);
C_l_delta_e = -((1.170e-4)+(2.794e-8)*(alpha*Ma)+(-1.16e-6)*delta_e+(-4.641e-11)*(alpha_deg^2*Ma^2*delta_e));
C_l_delta_r = (1.1441e-4)-(2.6824e-6)*alpha_deg-(3.5496e-6)*Ma+(5.5547e-8)*alpha_deg*Ma;
C_l_r = (3.82e-1)-(1.06e-1)*Ma+(1.94e-3)*alpha_deg-(8.15e-5)*(alpha_deg*Ma)+(1.45e-2)*Ma^2-(9.76e-6)*alpha_deg^2+(4.49e-8)*(alpha_deg*Ma)^2+(1.02e-3)*Ma^3-(2.70e-7)*alpha_deg^3+(3.56e-5)*Ma^4-(4.81e-7)*Ma^5;
C_l_p = (-2.99e-1)+(7.47e-2)*Ma+(1.38e-3)*alpha_deg-(8.78e-5)*(alpha_deg*Ma)-(9.13e-3)*Ma^2-(2.04e-4)*alpha_deg^2-(1.52e-7)*(alpha_deg*Ma)^2+(5.73e-4)*Ma^3-(3.86e-5)*alpha_deg^3-(1.79e-5)*Ma^4+(2.20e-7)*Ma^5;
%C_l_delta_a和C_l_delta_e的余项抵消了，C_m_delta_r的余项其实可以忽略
C_l_yuxiang = -5.0103e-19+(6.2723e-20)*alpha_deg+(2.3418e-20)*Ma-(3.4201e-21)*(alpha_deg*Ma);

end