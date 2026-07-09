function [C_n_beta,C_n_delta_a,C_n_delta_e,C_n_delta_r,C_n_p,C_n_r] = Coefficient_YawingMoment(alpha,Ma,delta_a,delta_e,delta_r)
%求偏航力矩系数各分量 2022.06.14 任斌
r2d = 180/pi;
% r2d = Coefficient_r2d(alpha*r2d);
alpha_deg = alpha * r2d;
C_n_beta=0+(6.9980e-4)*alpha_deg+(5.9115e-2)*Ma+(-7.5250e-5)*alpha_deg*Ma+(2.5160e-4)*alpha_deg^2+(-1.4824e-2)*Ma^2+(-2.1924e-7)*(alpha_deg*Ma)^2+(-1.0777e-4)*alpha_deg^3+(1.2692e-3)*Ma^3+(1.0707e-8)*(alpha_deg*Ma)^3+(-4.7098e-5)*Ma^4+(6.4284e-7)*Ma^5;
C_n_delta_a = -(1.30e-5)-(8.93e-8)*(alpha_deg*Ma)+(1.97e-6)*delta_a+(1.41e-11)*(alpha_deg^2*Ma^2*delta_a);
C_n_delta_e = -(-(1.30e-5)-(8.93e-8)*(alpha_deg*Ma)+(1.97e-6)*delta_e+(1.41e-11)*(alpha_deg^2*Ma^2*delta_e));
C_n_delta_r = -(5.28e-4)+(1.39e-5)*(alpha_deg)+(1.65e-5)*(Ma)-(3.13e-7)*(alpha_deg*Ma);
C_n_p = (3.68e-1)-(9.79e-2)*Ma+(7.61e-16)*alpha_deg+(1.24e-2)*Ma^2-(4.64e-16)*alpha_deg^2-(8.05e-4)*Ma^3+(1.01e-16)*alpha_deg^3+(2.57e-5)*Ma^4-(3.20e-7)*Ma^5;
C_n_r = -2.41+(5.96e-1)*Ma-(2.74e-3)*alpha_deg+(2.09e-4)*(alpha_deg*Ma)-(7.57e-2)*Ma^2+(1.15e-3)*alpha_deg^2-(6.53e-8)*(alpha_deg*Ma)^2+(4.90e-3)*Ma^3-(3.87e-4)*alpha_deg^3-(1.57e-4)*Ma^4+(1.96e-6)*Ma^5;
% C_n_delta_a的余项和C_n_delta_e的余项抵消了，C_n_delta_r的余项可以忽略
C_n_yuxiang = 2.85e-18-(3.59e-19)*alpha_deg-(1.26e-19)*Ma+(1.57e-20)*(alpha_deg*Ma);
end