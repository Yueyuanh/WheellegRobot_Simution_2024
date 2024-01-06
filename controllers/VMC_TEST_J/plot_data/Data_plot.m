% 读取文本文件
data1 = load('Rleg_L0.txt');
data2 = load('Rleg_d_L0.txt');

data3 = load('Lleg_L0.txt');
data4 = load('Lleg_d_L0.txt');

% 提取 x 和 y 数据
x = data1(:, 1);
y = data1(:, 2);

x2 = data2(:, 1);
y2 = data2(:, 2);

x3 = data3(:, 1);
y3 = data3(:, 2);

x4 = data4(:, 1);
y4 = data4(:, 2);

% 绘制曲线
subplot(2,1,1);
plot(x, y);  % 'o-' 表示使用圆圈标记连接线条
% 添加标题和标签
hold on;
plot(x3,y3);

legend('右腿腿长','左腿腿长');

title('腿长变化曲线图');
xlabel('时间');
ylabel('腿长');
% 显示网格
grid on;

subplot(2,1,2);
plot(x2, y2);  % 'o-' 表示使用圆圈标记连接线条
hold on;

dy=diff(y)./diff(x);
dx=x(1:length(dy));
plot(dx,dy);

% 添加标题和标签
title('腿长变化速度曲线图');
xlabel('时间');
ylabel('变化速度');

% 显示网格



legend('梯度估算','差分估算');

grid on;

