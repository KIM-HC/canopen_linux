clc
clear all

%% load data
% Time center:X center:Y center:Z x_axis:X x_axis:Y x_axis:Z y_axis:X y_axis:Y y_axis:Z
ar = load("mocap/mocap_2022_01_28_4_set0rotate.txt");


tot_tick = length(ar);

%% find start transform
% average quaternion and translation

%% express points in start axis

%% plot path
st = 1;   %% start tick
et = tot_tick;  %% end tick

figure(11)
subplot(1,1,1)
plot3(ar(st:et,2), ar(st:et,3), ar(st:et,4), 'LineWidth',1)
title('aruco 0')
hold on
axis equal
hold off
grid on


