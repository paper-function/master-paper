plot3(Scope_position_d(:,2),Scope_position_d(:,3),Scope_position_d(:,4));
grid on
hold on
plot3(Scope_position_target1(:,2),Scope_position_target1(:,3),Scope_position_target1(:,4),'o');
xlim([0 12e4]);
ylim([-1e5 1e5]);
zlim([-1e4 1e5]);