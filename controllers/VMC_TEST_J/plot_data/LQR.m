clear all


% ��ȡ�ı��ļ�
lqr_1_data = load('LQR_1.txt');
lqr_2_data = load('LQR_2.txt');
lqr_out_data=load('LQR_OUT.txt');


% ��ȡ����
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

% ��������
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

% ���ӱ���ͱ�ǩ
title('LQR״̬���� �ڽ�/�ڼ��ٶ�');
legend('leg-angle','leg-gyro','speed');
xlabel('ʱ��');
ylabel('rad rad/s^2');
% ��ʾ����
grid on;



subplot(2,2,2);
plot(lqr_1_t,foot_dis);
hold on
plot(lqr_2_t, foot_speed);
hold on
dt=diff(foot_dis)./diff(lqr_1_t);
dx=lqr_1_t(1:length(dt));
dt=dt*0.06;
plot(dx,dt);

% ���ӱ���ͱ�ǩ
title('LQR״̬���� ����/�ٶ�');
legend('foot-dis','foot-speed','speed');
xlabel('ʱ��');
ylabel('m m/s^2');
% ��ʾ����
grid on;


subplot(2,2,3);
hold on
plot(lqr_2_t,pitch);
hold on
plot(lqr_2_t,gyro_pitch);
hold on
dt_pitch=diff(pitch)./diff(lqr_2_t);
dx_pitch=lqr_2_t(1:length(dt_pitch));
plot(dx_pitch,dt_pitch);

% ���ӱ���ͱ�ǩ
title('LQR ״̬���� pitch gyro');
legend('pitch','gyro_p','speed');
xlabel('ʱ��');
ylabel('rad rad/s^2');
% ��ʾ����
grid on;

subplot(2,2,4);
plot(lqr_out_t, leg_r_T1);
hold on
plot(lqr_out_t,leg_r_T2);
hold on
plot(lqr_out_t,wheel_r_T*10);

% ���ӱ���ͱ�ǩ
title('LQR���');
legend('�ؽڵ��1','�ؽڵ��2','��챵��*10');
xlabel('ʱ��');
ylabel('rad/s');
% ��ʾ����
grid on;

