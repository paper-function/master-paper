function [C_D_alpha,C_D_delta_a,C_D_delta_e,C_D_delta_r] = Coefficient_Drag(alpha,Ma,delta_a,delta_e,delta_r)
%求阻力系数各分量 2022.06.07 任斌
r2d = 180/pi;
% r2d = Coefficient_r2d(alpha*r2d);
alpha_deg = alpha * r2d;
C_D_alpha=(8.717e-2)-Ma*(3.307e-2)+alpha_deg*(3.179e-3)-(alpha_deg*Ma)*(1.25e-4)+(Ma^2)*(5.036e-3)-(alpha_deg^2)*(1.1e-3)+((alpha_deg*Ma)^2)*(1.405e-7)-(Ma^3)*(3.658e-4)+(alpha_deg^3)*(3.175e-4)+(Ma^4)*(1.274e-5)-(alpha_deg^4)*(2.985e-5)-(Ma^5)*(1.705e-7)+(alpha_deg^5)*(9.766e-7);
C_D_delta_a = (4.5548e-4)+alpha_deg*(2.5411e-5)+Ma*(-1.1436e-4)+(-3.6417e-5)*delta_a+(alpha_deg*Ma*delta_a)*(-5.3015e-7)+(alpha_deg^2)*(3.2187e-6)+(Ma^2)*(3.014e-6)+(6.9629e-6)*(delta_a)^2+(2.1026e-12)*(alpha_deg*Ma*delta_a)^2;
C_D_delta_e = (4.5548e-4)+alpha_deg*(2.5411e-5)+Ma*(-1.1436e-4)+(-3.6417e-5)*delta_e+(alpha_deg*Ma*delta_e)*(-5.3015e-7)+(alpha_deg^2)*(3.2187e-6)+(Ma^2)*(3.014e-6)+(6.9629e-6)*(delta_e)^2+(2.1026e-12)*(alpha_deg*Ma*delta_e)^2;
C_D_delta_r=(7.50e-4)-(2.29e-5)*alpha_deg+Ma*(9.69e-5)-delta_r*(1.83e-6)+(alpha_deg*Ma*delta_r)*(9.13e-9)+(alpha_deg^2)*(8.76e-7)+(Ma^2)*(2.7e-6)+(delta_r^2)*(1.9701e-6)-((alpha_deg*Ma*delta_r)^2)*(1.7702e-11);
% C_D = C_D_alpha + C_D_delta_a + C_D_delta_e + C_D_delta_r;

% C_D0 = 8.717e-2 + 4.5548e-4 + 4.5548e-4 + 7.5e-4;
% C_D_alpha = 3.179e-3 + 2.5411e-5 + 2.5411e-5 - 2.29e-5;
% C_D_alpha2 = -1.1e-3 + 3.2187e-6 + 3.2187e-6 + 8.76e-7;
% C_D_alpha3 = 3.175e-4;
% C_D_alpha4 = -2.985e-5;
% C_D_alpha5 = 9.766e-7;
% C_D_Ma = -3.307e-2 - 1.1436e-4 - 1.1436e-4 - 9.69e-5;
% C_D_Ma2 = 5.036e-3 + 3.014e-6 + 3.014e-6 + 2.7e-6;
% C_D_Ma3 = -3.658e-4;
% C_D_Ma4 = 1.274e-5;
% C_D_Ma5 = -1.705e-7;
% C_D_delta_a = -3.6417e-5;
% C_D_delta_e = -3.6417e-5;
% C_D_delta_r = -1.83e-6;
% C_D = C_D0 + C_D_alpha*alpha_deg + C_D_alpha2*alpha_deg^2 + C_D_alpha3*alpha_deg^3 + C_D_alpha4*alpha_deg^4 + C_D_alpha5*alpha_deg^5  + C_D_Ma*Ma + C_D_Ma2*Ma^2 + C_D_Ma3*Ma^3 + C_D_Ma4*Ma^4 + C_D_Ma5*Ma^5 + C_D_delta_e*delta_e + C_D_delta_a*delta_a + C_D_delta_r*delta_r;

end