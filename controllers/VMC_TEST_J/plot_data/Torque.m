% ��ȡ�ı��ļ�
data_T = load('T.txt');

% ��ȡ x �� y ����
x_T = data_T(:, 1);
y_T = data_T(:, 2);


% ��������
plot(x_T, y_T);  % 'o-' ��ʾʹ��ԲȦ�����������
% ��ӱ���ͱ�ǩ
title('Ť���������ͼ');
xlabel('ʱ��');
ylabel('Ť��');
% ��ʾ����
grid on;

