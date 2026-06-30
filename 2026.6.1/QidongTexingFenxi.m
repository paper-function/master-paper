% 计算不同攻角和马赫数下的气动力系数，并画出特性曲线
clear all;
alpha_vec = 0:15;
Ma_vec = 6:20;
i = 1;
d2r = pi/180;
r2d = 180/pi;
beta = 0.1;
for alpha = alpha_vec
    j = 1;
    for Ma = Ma_vec
        [C_L_alpha,C_L_delta_a,C_L_delta_e] = Coefficient_Lift(alpha*d2r,Ma,0,0,0);
        C_L(i,j) = C_L_alpha+C_L_delta_a+C_L_delta_e;
        [C_D_alpha,C_D_delta_a,C_D_delta_e,C_D_delta_r] = Coefficient_Drag(alpha*d2r,Ma,0,0,0);
        C_D(i,j) = C_D_alpha+C_D_delta_e+C_D_delta_a+C_D_delta_r;
        [C_Y_beta,C_Y_delta_a,C_Y_delta_e,C_Y_delta_r] = Coefficient_Side(alpha*d2r,Ma,0,0,0);
        C_Y(i,j) = C_Y_beta*beta+C_Y_delta_a+C_Y_delta_e+C_Y_delta_r;
        j = j+1;
    end
    i = i+1;
end
C_D = C_D';
C_L = C_L';
C_Y = C_Y';
[alpha1,Ma1] = meshgrid(alpha_vec,Ma_vec);
[alpha_i,Ma_i] = meshgrid(0:0.5:15,6:0.5:20);
C_D_i = interp2(alpha1,Ma1,C_D,alpha_i,Ma_i,'Linear');
C_L_i = interp2(alpha1,Ma1,C_L,alpha_i,Ma_i,'Linear');
C_Y_i = interp2(alpha1,Ma1,C_Y,alpha_i,Ma_i,'Linear');
%figure1阻力特性曲线
figure(1)
yanse = ones(size(alpha_i,1),size(alpha_i,2),3);
for r = 1:size(alpha_i,1)
    for c = 1:size(alpha_i,2)
        yanse(r,c,:) = [0 1 1];
    end
end
surf(alpha_i,Ma_i,C_D_i,yanse);
xlabel('攻角(\circ)');
ylabel('马赫数Ma');
zlabel('阻力系数C_D');

%figure2升力特性曲线
figure(2)
surf(alpha_i,Ma_i,C_L_i,yanse);
xlabel('攻角(\circ)');
ylabel('马赫数Ma');
zlabel('升力系数C_L');

%figure3升阻比
figure(3)
surf(alpha_i,Ma_i,C_L_i./C_D_i,yanse);
xlabel('攻角(\circ)');
ylabel('马赫数Ma');
zlabel('升阻比');


%figure4侧力系数
figure(4)
surf(alpha_i,Ma_i,C_Y_i,yanse);
xlabel('攻角(\circ)');
ylabel('马赫数Ma');
zlabel('侧力系数C_Y');

