clc
clear all

% load data
date_num = "2022_03_04_";
set_num = 1;
% jt = load("joint/joint_" + date_num + set_num + ".csv");
jt = load("simulation/joint_" + date_num + set_num + ".csv");

% plot path
figure(13)
subplot(1,1,1)
plot(jt(:,3), 'LineWidth',1)
hold on
for module=2:4
    plot(jt(:,module * 2 + 1), 'LineWidth',1)
end
hold off
title('jt data steer')
grid on
