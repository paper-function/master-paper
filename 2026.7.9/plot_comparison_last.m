figure(1);
% subplot(5,1,1);
% load('CCI_DUI_last_k_c_0.95_d_0.mat');
% plot(time,Scope_e_alpha(:,2),'r');
% hold on
% load('CCI_DUI_last_k_c_0.95_d_0.25.mat');
% plot(time,Scope_e_alpha(:,2),'g');
% hold on
% load('CCI_DUI_last_k_c_0.95_d_0.5.mat');
% plot(time,Scope_e_alpha(:,2),'b');
% hold on
% load('CCI_DUI_last_k_c_0.95_d_0.75.mat');
% plot(time,Scope_e_alpha(:,2),'k');
% hold on
% load('CCI_DUI_last_k_c_0.95_d_1.mat');
% plot(time,Scope_e_alpha(:,2),'m');
% hold on
subplot(3,1,1);
load('Data_ADRC_nonoise_v3.mat');
doudong = 0.01*sin(2*pi*0.0001*time);
plot(time,Scope_e_alpha(:,2),'r');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_e_alpha(:,2),'b');
hold on
plot(time,Scope_e_alpha(:,2)+doudong,'g');
hold on
% legend("ADRC","CUDC\_ADRC");
% hold on
grid on;
xlabel('time(s)');
ylabel('e_{\alpha}/(°)')
title('e_{\alpha}');




subplot(3,1,2);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_e_beta(:,2),'r');
hold on

load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_e_beta(:,2),'b');
hold on
plot(time,Scope_e_beta(:,2)+doudong,'g');
hold on
legend("ADRC","CUDC\_ADRC");
grid on;
xlabel('time(s)');
ylabel('e_{\beta}/(°)')
title('e_{\beta}');



subplot(3,1,3);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_e_miu(:,2),'r');
hold on

load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_e_miu(:,2),'b');
hold on
plot(time,Scope_e_miu(:,2)+doudong,'g');
hold on
legend("ADRC","CUDC\_ADRC","TSMC");
grid on;
xlabel('time(s)');
ylabel('e_{\gamma}/(°)')
title('e_{\gamma}');

ax_inset = axes('Position', [0.2, 0.80, 0.1, 0.1]); 
plot(time,Scope_e_alpha(:,2), 'LineWidth', 1.5);
xlim([4, 5]);      % 放大 x 轴的 [2, 4] 区间
ylim([-0.02, 0.02]); % 放大 y 轴的对应区间
box on;            % 显示边框
title('局部放大细节');



figure(4);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_e_p(:,2),'r');
hold on
% load('Data_ADRC_noise.mat');
% plot(time,Scope_e_p(:,2),'g');
% hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_e_p(:,2),'b');
hold on
% load('Data_CCI_DUI_noise.mat');
% plot(time,Scope_e_p(:,2),'k');
grid on;
legend("ADRC","CUDC\_ADRC");
xlabel('time(s)');
ylabel('e_p/(°/s)')
title('e_{p}');



load('Data_ADRC_nonoise.mat');
plot(time,Scope_e_q(:,2),'r');
hold on
% load('Data_ADRC_noise.mat');
% plot(time,Scope_e_q(:,2),'g');
% hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_e_q(:,2),'b');
hold on
% load('Data_CCI_DUI_noise.mat');
% plot(time,Scope_e_q(:,2),'k');
grid on;
legend("ADRC","CUDC\_ADRC");
xlabel('time(s)');
ylabel('e_q/(°/s)')
title('e_{q}');




load('Data_ADRC_nonoise.mat');
plot(time,Scope_e_r(:,2),'r');
hold on
% load('Data_ADRC_noise.mat');
% plot(time,Scope_e_r(:,2),'g');
% hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_e_r(:,2),'b');
hold on
% load('Data_CCI_DUI_noise.mat');
% plot(time,Scope_e_r(:,2),'k');
grid on;
legend("ADRC","CUDC\_ADRC");
xlabel('time(s)');
ylabel('e_r/(°/s)')
title('e_{r}');



figure(7);
subplot(3,1,1)
load('Data_ADRC_nonoise.mat');
plot(time,Scope_delta_a(:,2),'r');
hold on
% load('Data_ADRC_noise.mat');
% plot(time,Scope_delta_a(:,2),'g');
% hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_delta_a(:,2),'b');
hold on
% load('Data_CCI_DUI_noise.mat');
% plot(time,Scope_delta_a(:,2),'k');
grid on;
legend("ADRC","CUDC\_ADRC");
xlabel('time(s)');
ylabel('{\delta}_{a}/(°)')
title('{\delta}_a');




subplot(3,1,2)
load('Data_ADRC_nonoise.mat');
plot(time,Scope_delta_e(:,2),'r');
hold on
% load('Data_ADRC_noise.mat');
% plot(time,Scope_delta_e(:,2),'g');
% hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_delta_e(:,2),'b');
hold on
% load('Data_CCI_DUI_noise.mat');
% plot(time,Scope_delta_e(:,2),'k');
grid on;
legend("ADRC","CUDC\_ADRC");
xlabel('time(s)');
ylabel('{\delta}_{e}/(°)')
title('{\delta}_e');



subplot(3,1,3)
load('Data_ADRC_nonoise.mat');
plot(time,Scope_delta_r(:,2),'r');
hold on
% load('Data_ADRC_noise.mat');
% plot(time,Scope_delta_r(:,2),'g');
% hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_delta_r(:,2),'b');
hold on
% load('Data_CCI_DUI_noise.mat');
% plot(time,Scope_delta_r(:,2),'k');
grid on;
legend("ADRC","CUDC\_ADRC");
xlabel('time(s)');
ylabel('{\delta}_{r}/(°)')
title('{\delta}_r');



figure(10);
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_CCI_FlowAngle(:,2),'r');
hold on
plot(time,Scope_DUI_FlowAngle(:,2),'g');
hold on 
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_CCI_FlowAngle(:,2),'b');
hold on
plot(time,Scope_DUI_FlowAngle(:,2),'k');
grid on
title('CCI/DUI');




figure(11);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_alpha_hat_error(:,2),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_alpha_hat_error(:,2),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_alpha_hat_error(:,2),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_alpha_hat_error(:,2),'k');
grid on;
title('e_{\alpha}hat');




figure(12);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_beta_hat_error(:,2),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_beta_hat_error(:,2),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_beta_hat_error(:,2),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_beta_hat_error(:,2),'k');
grid on;
title('e_{\beta}hat');


figure(13);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_miu_hat_error(:,2),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_miu_hat_error(:,2),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_miu_hat_error(:,2),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_miu_hat_error(:,2),'k');
grid on;
title('e_{\miu}hat');


figure(14);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_p_hat_error(:,2),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_p_hat_error(:,2),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_p_hat_error(:,2),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_p_hat_error(:,2),'k');
grid on;
title('e_{p}hat');

figure(15);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_q_hat_error(:,2),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_q_hat_error(:,2),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_q_hat_error(:,2),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_q_hat_error(:,2),'k');
grid on;
title('e_{q}hat');

figure(16);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_r_hat_error(:,2),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_r_hat_error(:,2),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_r_hat_error(:,2),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_r_hat_error(:,2),'k');
grid on;
title('e_{r}hat');

figure(17);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_phi_alpha_hat(:,3),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_phi_alpha_hat(:,3),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_phi_alpha_hat(:,3),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_phi_alpha_hat(:,3),'k');
grid on;
title('e_{\phi}_{\_}_{\alpha}');

figure(18);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_phi_beta_hat(:,3),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_phi_beta_hat(:,3),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_phi_beta_hat(:,3),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_phi_beta_hat(:,3),'k');
grid on;
title('e_{\phi}_{\_}_{\beta}');


figure(20);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_phi_miu_hat(:,3),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_phi_miu_hat(:,3),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_phi_miu_hat(:,3),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_phi_miu_hat(:,3),'k');
grid on;
title('e_{\phi}_{\_}_{\miu}');

figure(21);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_phi_p_hat(:,3),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_phi_p_hat(:,3),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_phi_p_hat(:,3),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_phi_p_hat(:,3),'k');
grid on;
title('e_{\phi}_{\_}_{p}');

figure(22);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_phi_q_hat(:,3),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_phi_q_hat(:,3),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_phi_q_hat(:,3),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_phi_q_hat(:,3),'k');
grid on;
title('e_{\phi}_{\_}_{q}');


figure(23);
load('Data_ADRC_nonoise.mat');
plot(time,Scope_phi_r_hat(:,3),'r');
hold on
load('Data_ADRC_noise.mat');
plot(time,Scope_phi_r_hat(:,3),'g');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_phi_r_hat(:,3),'b');
hold on
load('Data_CCI_DUI_noise.mat');
plot(time,Scope_phi_r_hat(:,3),'k');
grid on;
title('e_{\phi}_{\_}_{r}');


figure(24);
subplot(3,1,1);
load('Data_ADRC_nonoise_v3.mat');
plot(time,Scope_alpha(:,3),'k');
hold on
plot(time,Scope_alpha(:,2),'r--');
hold on
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_alpha(:,2),'b--');
hold on
grid on;
title('{\alpha}');
xlabel('time(s)');
ylabel('{\alpha}(°)')
legend('ref',"ADRC","CUDC\_ADRC");


subplot(3,1,2);
load('Data_ADRC_nonoise_v3.mat');
plot(time,Scope_beta(:,3),'k');
hold on
plot(time,Scope_beta(:,2),'r--');
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_beta(:,2),'b--');
hold on
grid on;
title('{\beta}');
xlabel('time(s)');
ylabel('{\beta}(°)')


subplot(3,1,3);
load('Data_ADRC_nonoise_v3.mat');
plot(time,Scope_miu(:,3),'k');
hold on
plot(time,Scope_miu(:,2),'r--');
load('Data_CCI_DUI_nonoise.mat');
plot(time,Scope_miu(:,2),'b--');
hold on
grid on;
title('{\gamma}');
xlabel('time(s)');
ylabel('{\gamma}(°)')

