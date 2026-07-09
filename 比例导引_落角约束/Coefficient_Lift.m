function [C_L_alpha,C_L_delta_a,C_L_delta_e] = Coefficient_Lift(alpha,Ma,delta_a,delta_e,delta_r)
%求升力系数各分量 2022.06.10 任斌
r2d = 180/pi;
% r2d = Coefficient_r2d(alpha*r2d);
alpha_deg = alpha * r2d;
C_L_alpha = (-8.19e-2)+(4.7e-2)*Ma+(1.86e-2)*alpha_deg-(4.73e-4)*(alpha_deg*Ma)-(9.19e-3)*Ma^2-(1.52e-4)*alpha_deg^2+(5.99e-7)*(alpha_deg*Ma)^2+(7.74e-4)*Ma^3+(4.08e-6)*alpha_deg^3-(2.93e-5)*Ma^4+(4.12e-7)*Ma^5;
C_L_delta_a = (-1.45e-5)+(1.01e-4)*alpha_deg+(7.10e-6)*Ma-(4.14e-4)*delta_a-(3.51e-6)*(alpha_deg*delta_a)+(4.70e-6)*(alpha_deg*Ma)+(8.72e-6)*(Ma*delta_a)-(1.70e-7)*(alpha_deg*Ma)*delta_a;
C_L_delta_e = (-1.45e-5)+(1.01e-4)*alpha_deg+(7.10e-6)*Ma-(4.14e-4)*delta_e-(3.51e-6)*(alpha_deg*delta_e)+(4.70e-6)*(alpha_deg*Ma)+(8.72e-6)*(Ma*delta_e)-(1.70e-7)*(alpha_deg*Ma)*delta_e;

% C_L = C_L_alpha + C_L_delta_a + C_L_delta_e;
% C_L_0 = -8.19e-2 + -1.45e-5 + -1.45e-5;
% C_L_alpha = 1.86e-2 + 1.01e-4 + 1.01e-4;
% C_L_alpha2 = -1.52e-4;
% C_L_alpha3 = 4.08e-6;
% C_L_alpha4 = -3.91e-7;
% C_L_alpha5 = 1.30e-8;
% C_L_Ma = 4.70e-2 + 7.10e-6 + 7.10e-6;
% C_L_Ma2 = -9.19e-3;
% C_L_Ma3 = 7.74e-4;
% C_L_Ma4 = -2.93e-5;
% C_L_Ma5 = 4.12e-7;
% C_L_delta_a = -4.14e-4;
% C_L_delta_e = -4.14e-4;
% C_L = C_L_0 + C_L_alpha*alpha_deg + C_L_alpha2*alpha_deg^2 + C_L_alpha3*alpha_deg^3 + C_L_alpha4*alpha_deg^4 + C_L_alpha5*alpha_deg^5  + C_L_Ma*Ma + C_L_Ma2*Ma^2 + C_L_Ma3*Ma^3 + C_L_Ma4*Ma^4 + C_L_Ma5*Ma^5 + C_L_delta_e*delta_e + C_L_delta_a*delta_a;

end