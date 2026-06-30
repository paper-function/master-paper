figure(1)
plot(time,C_L_d.*100,'r','Linewidth',1.5);
hold on;
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.5);   %设置虚网格
xlabel('时间（s）');
ylabel('升力系数摄动量（%）'); 
axis([-inf,inf,0,35])

figure(2)
plot(time,C_D_d.*100,'r','Linewidth',1.5);
hold on;
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.5);   %设置虚网格
xlabel('时间（s）');
ylabel('阻力系数摄动量（%）'); 
axis([-inf,inf,0,35])

figure(3)
plot(time,C_Y_d.*100,'r','Linewidth',1.5);
hold on;
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.5);   %设置虚网格
xlabel('时间（s）');
ylabel('侧力系数摄动量（%）'); 
axis([-inf,inf,0,35])

figure(4)
plot(time,C_l_d.*100,'r','Linewidth',1.5);
hold on;
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.5);   %设置虚网格
xlabel('时间（s）');
ylabel('滚转力矩系数摄动量（%）'); 
axis([-inf,inf,0,35])

figure(5)
plot(time,C_m_d.*100,'r','Linewidth',1.5);
hold on;
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.5);   %设置虚网格
xlabel('时间（s）');
ylabel('俯仰力矩系数摄动量（%）'); 
axis([-inf,inf,0,35])

figure(6)
plot(time,C_L_d.*100,'r','Linewidth',1.5);
hold on;
grid on;
set(gca,'GridLineStyle',':','GridColor','k','GridAlpha',0.5);   %设置虚网格
xlabel('时间（s）');
ylabel('偏航力矩系数摄动量（%）'); 
axis([-inf,inf,0,35])