% ��ȡ�ı��ļ�
imu_data = load('IMU.txt');
gyro_data = load('GYRO.txt');
accel_data=load('ACCEL.txt');


% ��ȡ����
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

% ��������
subplot(3,1,1);
plot(imu_t, yaw);
hold on
plot(imu_t,pitch);
hold on
plot(imu_t,roll);

% ��ӱ���ͱ�ǩ
title('����������ͼ');
legend('yaw','pitch','roll');
xlabel('ʱ��');
ylabel('IMU');
% ��ʾ����
grid on;

subplot(3,1,2);
plot(gyro_t, gyro_yaw);
hold on
plot(gyro_t,gyro_pitch);
hold on
plot(gyro_t,gyro_roll);

% ��ӱ���ͱ�ǩ
title('���ٶ�����ͼ');
legend('gyro_y','gyro_p','gyro_r');
xlabel('ʱ��');
ylabel('GYRO');
% ��ʾ����
grid on;

subplot(3,1,3);
plot(accel_t, accel_x);
hold on
plot(accel_t,accel_y);
hold on
plot(accel_t,accel_z);

% ��ӱ���ͱ�ǩ
title('���ٶ�����ͼ');
legend('accel_x','accel_y','accel_z');
xlabel('ʱ��');
ylabel('ACCEL');
% ��ʾ����
grid on;


