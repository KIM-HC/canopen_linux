clc
clear all

% load data
date_num = "2022_01_28_";
set_num = 3;
mv = load("mocap/mocap_" + date_num + set_num + ".txt");

dot = zeros(length(mv),2);
dt = mv(2,1) - mv(1,1);

a = 0.5;
fprintf('a = %f\n',a);

for tick=2:length(mv)
    dot(tick,1) = mv(tick,1);
    dot(tick,2) = a*dot(tick-1,2) + (1-a)*(norm(mv(tick,2:3)-mv(tick-1,2:3)) / dt);
end


% plot path
figure(11)
subplot(1,1,1)
plot(mv(:,1),mv(:,2), 'LineWidth',1)
hold on
plot(mv(:,1),mv(:,3), 'LineWidth',1)
hold off
legend({'X pos','Y pos'},'Location','best')
title("set" + set_num + " mocap data")
grid on

figure(12)
subplot(1,1,1)
plot(dot(:,1),dot(:,2), 'LineWidth',1)
hold on
legend({'X dot'},'Location','best')
title("set" + set_num + " mocap speed data")
grid on
