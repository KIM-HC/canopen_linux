clc
clear all

% load data
date_num = "2022_03_06_";
set_num = 1;
jt = load("joint_raw/joint_vel_" + date_num + set_num + ".csv");

dot = zeros(length(jt),9);

a = 0.995;
% a = 0.99;
fprintf('a = %f\n',a);

for tick=2:length(jt)
    dot(tick,1) = jt(tick,1);
    for idx=2:9
        dot(tick,idx) = a*dot(tick-1,idx) + (1-a)*jt(tick,idx);
    end
end


% plot path
figure(13)
subplot(1,1,1)
plot(dot(:,1), dot(:,2), 'LineWidth',1)
hold on
for idx=3:9
    plot(dot(:,1), dot(:,idx), 'LineWidth',1)
end
hold off
title('jt dot data')
grid on
