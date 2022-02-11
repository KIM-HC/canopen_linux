clc
clear all

%% load data
ar = load("aruco/aruco_2022_01_26_5.csv");
tot_tick = length(ar);
av = ones(tot_tick,4);
av(:,1:3) = (ar(:,2:4) + ar(:,9:11)) / 2;

%% find start transform
% average quaternion and translation
init_st = 10;
init_et = 100;
tot_quat = zeros(init_et - init_st + 1, 4);

for i = init_st:init_et
    idx = i - init_st + 1;
    tot_quat(idx,:) = [ar(i,8) ar(i,5) ar(i,6) ar(i,7)];
    tot_quat(idx+1,:) = [ar(i,15) ar(i,12) ar(i,13) ar(i,14)];
end
tot_quaternion = quaternion(tot_quat);
avg_quat = meanrot(tot_quaternion);
avg_rot = quat2rotm(avg_quat);
avg_translation = mean(av(init_st:init_et,1:3));


transform_inv = zeros(4,4);
transform_inv(1:3,1:3) = transpose(avg_rot);
transform_inv(1:3,4) = -transpose(avg_rot)*transpose(avg_translation);
transform_inv(4,4) = 1;

%% express points in start axis
avv = zeros(tot_tick,4);
for i = 1:tot_tick
    avv(i,:) = transpose(transform_inv * transpose(av(i,:)));
end

%% plot path
st = 1;   %% start tick
et = tot_tick;  %% end tick

% figure(11)
% subplot(2,1,1)
% plot3(ar(st:et,2), ar(st:et,3), ar(st:et,4), 'LineWidth',1)
% title('aruco 0')
% hold on
% axis equal
% hold off
% grid on
% 
% subplot(2,1,2)
% plot3(ar(st:et,9), ar(st:et,10), ar(st:et,11), 'LineWidth',1)
% title('aruco 1')
% hold on
% axis equal
% hold off
% grid on

figure(13)
plot3(av(st:et,1), av(st:et,2), av(st:et,3), 'LineWidth',1)
title('aruco avg')
hold on
axis equal
hold off
grid on

figure(14)
% plot3(avv(st:et,1), avv(st:et,2), avv(st:et,3), 'LineWidth',1)
plot(avv(st:et,1), avv(st:et,2), 'LineWidth',1)
title('aruco avg transformed')
hold on
axis equal
hold off
grid on




