function [C_Y_beta,C_Y_delta_a,C_Y_delta_e,C_Y_delta_r] = Coefficient_Side(alpha,Ma,delta_a,delta_e,delta_r)
%求侧力系数各分量 2022.06.13 任斌
r2d = 180/pi;
% r2d = Coefficient_r2d(alpha*r2d);
alpha_deg = alpha * r2d;
% C_Y_beta = 0-(2.9253e-1)*Ma+(-2.8803e-3)*alpha_deg-(2.8943e-4)*(alpha_deg*Ma)+(5.4822e-2)*Ma^2+(7.3535e-4)*alpha_deg^2-(4.6490e-9)*((alpha_deg*Ma^2)^2)-(2.0675e-8)*((alpha_deg^2*Ma)^2)+(4.6205e-6)*((alpha_deg*Ma)^2)+(2.6144e-11)*((alpha_deg^2*Ma^2)^2)-(4.3203e-3)*Ma^3-(3.7405e-4)*alpha_deg^3+(1.5495e-4)*Ma^4+(2.8183e-5)*alpha_deg^4-(2.0829e-6)*Ma^5-(5.2083e-7)*alpha_deg^5;
C_Y_beta = 0-(2.9253e-1)*Ma+(2.8803e-3)*alpha_deg-(2.8943e-4)*(alpha_deg*Ma)+(5.4822e-2)*Ma^2+(7.3535e-4)*alpha_deg^2-(4.6490e-9)*((alpha_deg*Ma^2)^2)-(2.0675e-8)*((alpha_deg^2*Ma)^2)+(4.6205e-6)*((alpha_deg*Ma)^2)+(2.6144e-11)*((alpha_deg^2*Ma^2)^2)-(4.3203e-3)*Ma^3-(3.7405e-4)*alpha_deg^3+(1.5495e-4)*Ma^4+(2.8183e-5)*alpha_deg^4-(2.0829e-6)*Ma^5-(5.2083e-7)*alpha_deg^5;
C_Y_delta_a = (-1.02e-6)-(1.12e-7)*alpha_deg+(4.48e-7)*Ma+(2.27e-7)*delta_a+(4.11e-9)*(alpha_deg*Ma)*delta_a+(2.82e-9)*alpha_deg^2-(2.36e-8)*Ma^2-(5.04e-8)*delta_a^2+(4.50e-14)*(alpha_deg*Ma*delta_a)^2;
C_Y_delta_e = -((-1.02e-6)-(1.12e-7)*alpha_deg+(4.48e-7)*Ma+(2.27e-7)*delta_e+(4.11e-9)*(alpha_deg*Ma)*delta_e+(2.82e-9)*alpha_deg^2-(2.36e-8)*Ma^2-(5.04e-8)*delta_e^2+(4.50e-14)*(alpha_deg*Ma*delta_e)^2);
C_Y_delta_r = (-1.43e-18)+(4.86e-20)*alpha_deg+(1.86e-19)*Ma+(3.84e-4)*delta_r-(1.17e-5)*(alpha_deg*delta_r)-(1.07e-5)*(Ma*delta_r)+(2.60e-7)*(alpha_deg*Ma*delta_r);

% C_Y_0 = -1.02e-6 + 1.02e-6 + -1.43e-18;
% C_Y_alpha = 2.8803e-3 + -1.12e-7 + 1.12e-7 + 4.86e-20;
% C_Y_alpha2 = 7.3535e-4 + 2.82e-9 + -2.82e-9;
% C_Y_alpha3 = -3.7405e-4;
% C_Y_alpha4 = 2.8183e-5;
% C_Y_alpha5 = -5.2083e-7;
% C_Y_Ma = -2.9253e-1 + 4.48e-7 + -4.48e-7 + 1.86e-19;
% C_Y_Ma2 = 5.4822e-2 + -2.36e-8 + 2.36e-8;
% C_Y_Ma3 = -4.3203e-3;
% C_Y_Ma4 = 1.5495e-4;
% C_Y_Ma5 = -2.0829e-6;
% C_Y_delta_e = 2.27e-7;
% C_Y_delta_a = -2.27e-7;
% C_Y_delta_r = 3.84e-4;
% C_Y = C_Y_0 + C_Y_alpha*alpha_deg + C_Y_alpha2*alpha_deg^2 + C_Y_alpha3*alpha_deg^3 + C_Y_alpha4*alpha_deg^4 + C_Y_alpha5*alpha_deg^5  + C_Y_Ma*Ma + C_Y_Ma2*Ma^2 + C_Y_Ma3*Ma^3 + C_Y_Ma4*Ma^4 + C_Y_Ma5*Ma^5 + C_Y_delta_e*delta_e + C_Y_delta_a*delta_a + C_Y_delta_r*delta_r;

end