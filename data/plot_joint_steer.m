clc
clear all

% 21 set-0
% 22 set-1
% 23 set-2
% 24 set-3 trial 1
% 25 set-3 trial 2


% load data
date_num = "2022_03_07_";
set_num = 25;
% jt = load("joint/joint_" + date_num + set_num + ".csv");
jt = load("joint_raw/joint_" + date_num + set_num + ".csv");

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
legend({'0','1','2','3'})
grid on
