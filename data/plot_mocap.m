clc
clear all

% load data
% ar = load("mocap/equal_mocap_2022_01_28_0.txt");
ar = load("mocap/equal_mocap_2022_01_28_1.txt");

center_index = 2;

tot_tick = length(ar);

% find start transform
% average quaternion and translation

% express points in start axis

% plot path
st = 1;   %% start tick
et = tot_tick;  %% end tick

figure(11)
subplot(1,1,1)
plot3(ar(st:et,center_index), ar(st:et,center_index+1), ar(st:et,center_index+2), 'LineWidth',1)
title('mocap')
hold on
axis equal
hold off
grid on


