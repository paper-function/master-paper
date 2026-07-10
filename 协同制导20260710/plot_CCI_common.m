figure(1);
plot(time,Scope_alpha(:,2),'r');
hold on 
plot(time,Scope_alpha_CCI_0d9(:,2),'g');
hold on 
plot(time,Scope_alpha_CCI_0d8(:,2),'b');
grid on;
title('alpha');


figure(2);
plot(time,Scope_e_alpha(:,2),'r');
hold on
plot(time,Scope_e_alpha_0d9(:,2),'g');
hold on
plot(time,Scope_e_alpha_0d8(:,2),'b');
grid on;
title('e_alpha');


figure(3);
plot(time,Scope_delta_a(:,2),'r');
hold on; 
plot(time,Scope_delta_a_0d9(:,2),'g');
hold on; 
plot(time,Scope_delta_a_0d8(:,2),'b');
grid on;
title('delta_a');

figure(4);
plot(time,Scope_delta_e(:,2),'r');
hold on;
plot(time,Scope_delta_e_0d9(:,2),'g');
hold on;
plot(time,Scope_delta_e_0d8(:,2),'b');
grid on;
title('delta_e');

figure(5);
plot(time,Scope_p(:,2),'r');
hold on;
plot(time,Scope_p_0d9(:,2),'g');
hold on;
plot(time,Scope_p_0d8(:,2),'b');
grid on;
title('p');

% figure(6);
% plot(time,Scope_q(:,2));
% hold on;
% plot(time,Scope_q_CCI(:,2));
% grid on;
% title('q');


