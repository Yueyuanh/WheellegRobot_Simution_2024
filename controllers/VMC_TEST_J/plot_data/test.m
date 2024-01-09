% 设定文件夹路径和文件名
folderPath = 'plot_data';
fileName = 'LQR.txt';
 
% 进入文件夹
cd(folderPath);

while true
    % 获取文件信息
    fileInfo = dir(fileName);
    
    % 检查文件的修改时间是否有变化
    if fileInfo.datenum > lastModifiedTime
        lastModifiedTime = fileInfo.datenum;
        
        % 打开文件
        fileID = fopen(fileName, 'r');
        
        % 读取文件内容
        fileContent = fread(fileID, inf, 'uint8=>char')';
        
        % 关闭文件
        fclose(fileID);
        
        % 处理文件内容，可以根据需要进行其他操作
        disp('文件内容：');
        disp(fileContent);
    end
    
    % 等待一段时间再检查文件变化，可以根据需要调整等待时间
    pause(1);
end
