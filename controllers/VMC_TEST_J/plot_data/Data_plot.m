% ��ȡ�ı��ļ�
data1 = load('Rleg_L0.txt');
data2 = load('Rleg_d_L0.txt');

data3 = load('Lleg_L0.txt');
data4 = load('Lleg_d_L0.txt');

% ��ȡ x �� y ����
x = data1(:, 1);
y = data1(:, 2);

x2 = data2(:, 1);
y2 = data2(:, 2);

x3 = data3(:, 1);
y3 = data3(:, 2);

x4 = data4(:, 1);
y4 = data4(:, 2);

% ��������
subplot(2,1,1);
plot(x, y);  % 'o-' ��ʾʹ��ԲȦ�����������
% ��ӱ���ͱ�ǩ
hold on;
plot(x3,y3);

legend('�����ȳ�','�����ȳ�');

title('�ȳ��仯����ͼ');
xlabel('ʱ��');
ylabel('�ȳ�');
% ��ʾ����
grid on;

subplot(2,1,2);
plot(x2, y2);  % 'o-' ��ʾʹ��ԲȦ�����������
hold on;

dy=diff(y)./diff(x);
dx=x(1:length(dy));
plot(dx,dy);

% ��ӱ���ͱ�ǩ
title('�ȳ��仯�ٶ�����ͼ');
xlabel('ʱ��');
ylabel('�仯�ٶ�');

% ��ʾ����



legend('�ݶȹ���','��ֹ���');

grid on;

