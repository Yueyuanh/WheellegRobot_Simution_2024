clear all

%% 读取文本文件
lqr_1_data = load('LQR_1.txt');
lqr_2_data = load('LQR_2.txt');
lqr_out_data=load('LQR_OUT.txt');
set_state=load('Set_state.txt');

%% 提取数据
lqr_1_t = lqr_1_data(:, 1);
lqr_2_t = lqr_2_data(:, 1);
lqr_out_t = lqr_out_data(:, 1);
 
leg_angle = lqr_1_data(:, 2);
leg_gyro = lqr_1_data(:, 3);
foot_dis = lqr_1_data(:, 4);

foot_speed=lqr_2_data(:,2);
pitch=lqr_2_data(:,3);
gyro_pitch=lqr_2_data(:,4);

leg_r_T1=lqr_out_data(:,2);
leg_r_T2=lqr_out_data(:,3);
wheel_r_T=lqr_out_data(:,4);

distance_set=set_state(:,2);
yaw_set=set_state(:,3);

%% 绘制曲线
figure;
%% 摆角
subplot(2,2,1);
plot(lqr_1_t, leg_angle);
hold on
plot(lqr_1_t,leg_gyro);
hold on 

dt_leg=diff(leg_angle)./diff(lqr_1_t);
dx_angle=lqr_1_t(1:length(dt_leg));
% 
% dt_gyro=diff(dt_leg)./diff(dx_angle);
% dx_gyro=dx_angle(1:length(dt_gyro));

plot(dx_angle,dt_leg);

% 添加标题和标签
title('LQR状态变量 摆角/摆加速度');
legend('leg-angle','leg-gyro','leg-gyro-check');
xlabel('时间');
ylabel('rad rad/s');
% 显示网格
grid on;


%% 距离速度
subplot(2,2,2);
plot(lqr_1_t,foot_dis);
hold on
plot(lqr_2_t, foot_speed);
hold on
dt=diff(foot_dis)./diff(lqr_1_t);
dx=lqr_1_t(1:length(dt));
plot(dx,dt);
hold on
plot(lqr_1_t,distance_set)

% 添加标题和标签
title('LQR状态变量 距离/速度');
legend('foot-dis','foot-speed','foot-speed-check');
xlabel('时间');
ylabel('m m/s');
% 显示网格
grid on;

%% pitch gyro
subplot(2,2,3);
hold on
plot(lqr_2_t,pitch);
hold on
plot(lqr_2_t,gyro_pitch);
hold on
dt_pitch=diff(pitch)./diff(lqr_2_t);
dx_pitch=lqr_2_t(1:length(dt_pitch));
plot(dx_pitch,dt_pitch);

% 添加标题和标签
title('LQR 状态变量 pitch gyro');
legend('pitch','gyro_p','gyro-check');
xlabel('时间');
ylabel('rad rad/s');
% 显示网格
grid on;

%% 电机输出
subplot(2,2,4);
plot(lqr_out_t, leg_r_T1);
hold on
plot(lqr_out_t,leg_r_T2);
hold on
plot(lqr_out_t,wheel_r_T*10);

% 添加标题和标签
title('LQR输出');
legend('关节电机1','关节电机2','轮毂电机*10');
xlabel('时间');
ylabel('rad/s');
% 显示网格
grid on;


