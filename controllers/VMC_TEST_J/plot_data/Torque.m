% 读取文本文件
data_T = load('T.txt');

% 提取 x 和 y 数据
x_T = data_T(:, 1);
y_T = data_T(:, 2);


% 绘制曲线
plot(x_T, y_T);  % 'o-' 表示使用圆圈标记连接线条
% 添加标题和标签
title('扭矩输出曲线图');
xlabel('时间');
ylabel('扭矩');
% 显示网格
grid on;

