% �趨�ļ���·�����ļ���
folderPath = 'plot_data';
fileName = 'LQR.txt';
 
% �����ļ���
cd(folderPath);

while true
    % ��ȡ�ļ���Ϣ
    fileInfo = dir(fileName);
    
    % ����ļ����޸�ʱ���Ƿ��б仯
    if fileInfo.datenum > lastModifiedTime
        lastModifiedTime = fileInfo.datenum;
        
        % ���ļ�
        fileID = fopen(fileName, 'r');
        
        % ��ȡ�ļ�����
        fileContent = fread(fileID, inf, 'uint8=>char')';
        
        % �ر��ļ�
        fclose(fileID);
        
        % �����ļ����ݣ����Ը�����Ҫ������������
        disp('�ļ����ݣ�');
        disp(fileContent);
    end
    
    % �ȴ�һ��ʱ���ټ���ļ��仯�����Ը�����Ҫ�����ȴ�ʱ��
    pause(1);
end
