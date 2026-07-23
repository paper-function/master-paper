function [C_m_alpha,C_m_delta_a,C_m_delta_e,C_m_delta_r,C_m_q] = Coefficient_PitchMoment_1(alpha,Ma,delta_a,delta_e,delta_r)
%求俯仰力矩系数各分量 2022.06.13 任斌
r2d = 180/pi;
% r2d = Coefficient_r2d(alpha*r2d);
alpha_deg = alpha * r2d;
% C_m_alpha1 = (-2.192e-2)+(7.739e-3)*Ma-(2.260e-3)*alpha_deg+(1.808e-4)*(alpha_deg*Ma)-(8.849e-4)*(Ma^2)+(2.616e-4)*(alpha_deg^2)-(2.88e-7)*(alpha_deg*Ma)^2+(4.617e-5)*(Ma^3)-(7.887e-5)*alpha_deg^3-(1.143e-6)*Ma^4+(8.288e-6)*(alpha_deg^4)+(1.082e-8)*Ma^5-(2.789e-7)*alpha_deg^5;
% C_m_delta_a = (2.89e-4)+(4.48e-6)*(alpha_deg)-(5.87e-6)*(Ma)+(9.72e-8)*(alpha_deg*Ma);
% C_m_delta_e = (2.89e-4)+(4.48e-6)*(alpha_deg)-(5.87e-6)*(Ma)+(9.72e-8)*(alpha_deg*Ma);
% C_m_delta_r = (1.43e-7)*delta_r^3-(4.77e-22)*delta_r^4-(3.38e-10)*delta_r^5+(2.63e-24)*delta_r^6;
% % C_m_q = -1.36+(3.86e-1)*Ma+(-7.85e-4)*alpha_deg+(1.40e-4)*alpha_deg*Ma-(5.42e-2)*Ma^2+(2.36e-3)*alpha_deg^2-(1.95e-6)*(alpha_deg*Ma)^2+(3.80e-3)*Ma^3-(1.48e-3)*alpha_deg^3-(1.30e-4)*Ma^4+(1.69e-4)*alpha_deg^4+(1.71e-6)*Ma^5-(5.93e-6)*alpha_deg^5;
% C_m_q = -1.36+(3.86e-1)*Ma+(7.85e-4)*alpha_deg+(1.40e-4)*alpha_deg*Ma-(5.42e-2)*Ma^2+(2.36e-3)*alpha_deg^2-(1.95e-6)*(alpha_deg*Ma)^2+(3.80e-3)*Ma^3-(1.48e-3)*alpha_deg^3-(1.30e-4)*Ma^4+(1.69e-4)*alpha_deg^4+(1.71e-6)*Ma^5-(5.93e-6)*alpha_deg^5;
% C_m_yuxiang = (-5.67e-5)-(6.59e-5)*alpha_deg-(1.51e-6)*Ma-(4.46e-6)*(alpha_deg*Ma)+((-5.67e-5)-(6.59e-5)*alpha_deg-(1.51e-6)*Ma-(4.46e-6)*(alpha_deg*Ma))+(-2.79e-5)*alpha_deg-(5.89e-8)*alpha_deg^2+(1.58e-3)*Ma^2+(6.42e-8)*alpha_deg^3-(6.69e-4)*Ma^3-(2.10e-8)*alpha_deg^4+(1.05e-4)*Ma^4+(3.14e-9)*alpha_deg^5-(7.74e-6)*Ma^5-(2.8e-10)*alpha_deg^6+(2.70e-7)*Ma^6+(5.74e-12)*alpha_deg^7-(3.58e-9)*Ma^7;
% C_m_alpha = C_m_alpha1 + C_m_yuxiang;

C_m_0 = -2.192e-2 + -5.67e-5 + -5.67e-5;
C_m_alpha = -2.26e-3 + -6.59e-5 + -6.59e-5;
C_m_alpha2 = 2.616e-4;
C_m_alpha3 = -7.887e-5;
C_m_alpha4 = 8.288e-6;
C_m_alpha5 = -2.789e-7;
C_m_Ma = 7.739e-3 + -1.51e-6 + -1.51e-6;
C_m_Ma2 = -8.849e-4;
C_m_Ma3 = 4.617e-5;
C_m_Ma4 = -1.143e-6;
C_m_Ma5 = 1.082e-8;

C_m_delta_e = (2.89e-4)+(4.48e-6)*(alpha_deg)-(5.87e-6)*(Ma)+(9.72e-8)*(alpha_deg*Ma);
C_m_delta_a = (2.89e-4)+(4.48e-6)*(alpha_deg)-(5.87e-6)*(Ma)+(9.72e-8)*(alpha_deg*Ma);
C_m_delta_r = 0;
C_m_q = -1.36+(3.86e-1)*Ma+(7.85e-4)*alpha_deg+(1.40e-4)*alpha_deg*Ma-(5.42e-2)*Ma^2+(2.36e-3)*alpha_deg^2-(1.95e-6)*(alpha_deg*Ma)^2+(3.80e-3)*Ma^3-(1.48e-3)*alpha_deg^3-(1.30e-4)*Ma^4+(1.69e-4)*alpha_deg^4+(1.71e-6)*Ma^5-(5.93e-6)*alpha_deg^5;
C_m1 = C_m_0 + C_m_alpha*alpha_deg + C_m_alpha2*alpha_deg^2 + C_m_alpha3*alpha_deg^3 + C_m_alpha4*alpha_deg^4 + C_m_alpha5*alpha_deg^5 + C_m_Ma*Ma + C_m_Ma2*Ma^2 + C_m_Ma3*Ma^3 + C_m_Ma4*Ma^4 + C_m_Ma5*Ma^5;

end
