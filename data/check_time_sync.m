clc
clear all

% load data
% Time center:X center:Y center:Z x_axis:X x_axis:Y x_axis:Z y_axis:X y_axis:Y y_axis:Z
mv = load("mocap/mocap_2022_01_28_0.txt");
jt = load("joint/joint_2022_01_28_0.csv");

mv_start = [2416, 3069, 9282, 10002];
jt_start = [2883, 4506, 23082, 24908];
delta_time = 0.0;

for i=1:4
    mv(mv_start(i),1) - jt(jt_start(i),1)
    delta_time = delta_time + mv(mv_start(i),1) - jt(jt_start(i),1);
end
delta_time = delta_time / 4.0
for i = 1:length(jt)
    jt(i,1) = jt(i,1) + delta_time;
end

mv_st= 1;
jt_st = 1;
compare_st_time = max([mv(1,1), jt(1,1)]) + 0.5;
compare_end_time = min([mv(length(mv),1),jt(length(jt),1)]) - 0.5;

while (mv(mv_st,1) < compare_st_time)
    mv_st = mv_st + 1;
end
while (jt(jt_st,1) < compare_st_time)
    jt_st = jt_st + 1;
end

mv_ed = mv_st;
jt_ed = jt_st;
while (mv(mv_ed,1) < compare_end_time)
    mv_ed = mv_ed + 1;
end
while (jt(jt_ed,1) < compare_end_time)
    jt_ed = jt_ed + 1;
end

% plot path
figure(14)
mv1 = subplot(2,1,1);
plot(mv(mv_st:mv_ed,1),mv(mv_st:mv_ed,2), 'LineWidth',1)
title('mv time')
legend({'mv x'},'Location','best')
grid on
jt1 = subplot(2,1,2);
plot(jt(jt_st:jt_ed,1),jt(jt_st:jt_ed,2), 'LineWidth',1)
title('jt time')
hold on
plot(jt(jt_st:jt_ed,1),jt(jt_st:jt_ed,4), 'LineWidth',1)
plot(jt(jt_st:jt_ed,1),jt(jt_st:jt_ed,6), 'LineWidth',1)
plot(jt(jt_st:jt_ed,1),jt(jt_st:jt_ed,8), 'LineWidth',1)
legend({'set 0','set 1','set 2','set 3'},'Location','best')
grid on
hold off
linkaxes([mv1,jt1],'x')

