lqr_r=load('LQR_R.txt');
lqr_l=load('LQR_L.txt');
lqr_wr=load('LQR_WR.txt');
lqr_wl=load('LQR_WL.txt');

lqr_t=lqr_r(:, 1);
%%
%读取数据
lqr_r_00=lqr_r(:,2);
lqr_r_01=lqr_r(:,3);
lqr_r_02=lqr_r(:,4);
lqr_r_03=lqr_r(:,5);
lqr_r_04=lqr_r(:,6);
lqr_r_05=lqr_r(:,7);

lqr_r_10=lqr_wr(:,2);
lqr_r_11=lqr_wr(:,3);
lqr_r_12=lqr_wr(:,4);
lqr_r_13=lqr_wr(:,5);
lqr_r_14=lqr_wr(:,6);
lqr_r_15=lqr_wr(:,7);

lqr_l_00=lqr_l(:,2);
lqr_l_01=lqr_l(:,3);
lqr_l_02=lqr_l(:,4);
lqr_l_03=lqr_l(:,5);
lqr_l_04=lqr_l(:,6);
lqr_l_05=lqr_l(:,7);

lqr_l_10=lqr_wl(:,2);
lqr_l_11=lqr_wl(:,3);
lqr_l_12=lqr_wl(:,4);
lqr_l_13=lqr_wl(:,5);
lqr_l_14=lqr_wl(:,6);
lqr_l_15=lqr_wl(:,7);

%%
%右腿关节扭矩

subplot(2,2,1);

plot(lqr_t,lqr_r_00);
hold on;
plot(lqr_t,lqr_r_01);
hold on;
plot(lqr_t,lqr_r_02);
hold on;
plot(lqr_t,lqr_r_03);
hold on;
plot(lqr_t,lqr_r_04);
hold on;
plot(lqr_t,lqr_r_05);
hold on;
plot(lqr_t,lqr_r_00+lqr_r_01+lqr_r_02+lqr_r_03+lqr_r_04+lqr_r_05,"o-");

% 添加标题和标签
title('LQR 右侧关节Tp输出');
legend('leg_angle','leg_gyro','distance','speed','pitch','gyro');
xlabel('时间');
grid on;

%%
%右轮
subplot(2,2,3);
plot(lqr_t,lqr_r_10);
hold on;
plot(lqr_t,lqr_r_11);
hold on;
plot(lqr_t,lqr_r_12);
hold on;
plot(lqr_t,lqr_r_13);
hold on;
plot(lqr_t,lqr_r_14);
hold on;
plot(lqr_t,lqr_r_15);
hold on;
plot(lqr_t,lqr_r_10+lqr_r_11+lqr_r_12+lqr_r_13+lqr_r_14+lqr_r_15,"o-");

% 添加标题和标签
title('LQR 右侧轮T输出');
legend('leg_angle','leg_gyro','distance','speed','pitch','gyro');
xlabel('时间');
grid on;

%%
%左腿关节扭矩
subplot(2,2,2);

plot(lqr_t,lqr_l_00);
hold on;
plot(lqr_t,lqr_l_01);
hold on;
plot(lqr_t,lqr_l_02);
hold on;
plot(lqr_t,lqr_l_03);
hold on;
plot(lqr_t,lqr_l_04);
hold on;
plot(lqr_t,lqr_l_05);
hold on;
plot(lqr_t,lqr_l_00+lqr_l_01+lqr_l_02+lqr_l_03+lqr_l_04+lqr_l_05,"o-");

% 添加标题和标签
title('LQR 左侧关节Tp输出');
legend('leg_angle','leg_gyro','distance','speed','pitch','gyro');
xlabel('时间');
grid on;

%%
%左轮
subplot(2,2,4);
plot(lqr_t,lqr_l_10);
hold on;
plot(lqr_t,lqr_l_11);
hold on;
plot(lqr_t,lqr_l_12);
hold on;
plot(lqr_t,lqr_l_13);
hold on;
plot(lqr_t,lqr_l_14);
hold on;
plot(lqr_t,lqr_l_15);
hold on;
plot(lqr_t,lqr_l_10+lqr_l_11+lqr_l_12+lqr_l_13+lqr_l_14+lqr_l_15,"o-");

% 添加标题和标签
title('LQR 左侧轮T输出');
legend('leg_angle','leg_gyro','distance','speed','pitch','gyro');
xlabel('时间');
grid on;



















