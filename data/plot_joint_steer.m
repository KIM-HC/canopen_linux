clc
clear all

% load data
date_num = "2022_01_28_";
set_num = 2;
jt = load("joint/joint_" + date_num + set_num + ".csv");

% plot path
figure(13)
subplot(1,1,1)
plot(jt(:,set_num * 2 + 3), 'LineWidth',1)
title('jt data steer')
grid on
