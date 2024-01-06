% 读取文本文件
imu_data = load('IMU.txt');
gyro_data = load('GYRO.txt');
accel_data=load('ACCEL.txt');


% 提取数据
imu_t = imu_data(:, 1);
gyro_t = gyro_data(:, 1);
accel_t = accel_data(:, 1);

yaw = imu_data(:, 2);
pitch = imu_data(:, 3);
roll = imu_data(:, 4);

gyro_yaw=gyro_data(:,2);
gyro_pitch=gyro_data(:,3);
gyro_roll=gyro_data(:,4);

accel_x=accel_data(:,2);
accel_y=accel_data(:,3);
accel_z=accel_data(:,4);

% 绘制曲线
subplot(3,1,1);
plot(imu_t, yaw);
hold on
plot(imu_t,pitch);
hold on
plot(imu_t,roll);

% 添加标题和标签
title('陀螺仪曲线图');
legend('yaw','pitch','roll');
xlabel('时间');
ylabel('IMU');
% 显示网格
grid on;

subplot(3,1,2);
plot(gyro_t, gyro_yaw);
hold on
plot(gyro_t,gyro_pitch);
hold on
plot(gyro_t,gyro_roll);

% 添加标题和标签
title('角速度曲线图');
legend('gyro_y','gyro_p','gyro_r');
xlabel('时间');
ylabel('GYRO');
% 显示网格
grid on;

subplot(3,1,3);
plot(accel_t, accel_x);
hold on
plot(accel_t,accel_y);
hold on
plot(accel_t,accel_z);

% 添加标题和标签
title('加速度曲线图');
legend('accel_x','accel_y','accel_z');
xlabel('时间');
ylabel('ACCEL');
% 显示网格
grid on;


