#include <plot.h>
#include <string.h>
#include <stdio.h>


void plotInit()
{
    removeFile("Rleg_d_L0");
    removeFile("Rleg_L0");
    removeFile("Lleg_d_L0");
    removeFile("Lleg_L0");
    removeFile("IMU");
    removeFile("GYRO");
    removeFile("ACCEL");
    removeFile("T");
    removeFile("LQR_1");
    removeFile("LQR_2");
    removeFile("LQR_OUT");
    removeFile("LQR_R");//void LQR_log()
    removeFile("LQR_L");
    removeFile("LQR_WR");
    removeFile("LQR_WL");
}

void plotFile(const char *fileName,float input_x,float input_y)
 {
    FILE *file;
    char fileFolder[100] = "plot_data/";
    char fileFormat[10] = ".txt";
    int fileNum;
    strcat(fileFolder, fileName);
    strcat(fileFolder, fileFormat);

    // 以写入模式打开文件，如果文件不存在则创建新文件
    if ((file = fopen(fileFolder, "a")) != NULL) {
        fprintf(file, "%lf %lf\n", input_x, input_y);
        fclose(file);
    } else {
        printf("无法创建文件 %s。\n", fileName);
    }

}

void plotFile3(const char *fileName,float input_t,float input_x,float input_y,float input_z)
 {
    FILE *file;
    char fileFolder[100] = "plot_data/";
    char fileFormat[10] = ".txt";
    int fileNum;
    strcat(fileFolder, fileName);
    strcat(fileFolder, fileFormat);

    // 以写入模式打开文件，如果文件不存在则创建新文件
    if ((file = fopen(fileFolder, "a")) != NULL) {
        fprintf(file, "%lf %lf %1f %1f\n", input_t, input_x,input_y,input_z);
        fclose(file);
    } else {
        printf("无法创建文件 %s。\n", fileName);
    }

}
void plotFile6(const char *fileName,float input_t,float input_x,float input_y,float input_z,float input_a,float input_b,float input_c)
 {
    FILE *file;
    char fileFolder[100] = "plot_data/";
    char fileFormat[10] = ".txt";
    int fileNum;
    strcat(fileFolder, fileName);
    strcat(fileFolder, fileFormat);

    // 以写入模式打开文件，如果文件不存在则创建新文件
    if ((file = fopen(fileFolder, "a")) != NULL) {
        fprintf(file, "%lf %lf %1f %1f %1f %1f %1f\n", input_t, input_x,input_y,input_z,input_a,input_b,input_c);
        fclose(file);
    } else {
        printf("无法创建文件 %s。\n", fileName);
    }

}
void numFile(const char *fileName,float input_num)
 {
    FILE *file;
    char fileFolder[100] = "plot_data/";
    char fileFormat[10] = ".txt";
    strcat(fileFolder, fileName);
    strcat(fileFolder, fileFormat);

    // 以写入模式打开文件，如果文件不存在则创建新文件
    if ((file = fopen(fileFolder, "w")) != NULL) {
        fprintf(file, "%f \n", input_num);
        fclose(file);
    } else {
        printf("无法创建文件 %s。\n", fileName);
    }

}

int readNum(const char *fileName)
{
    FILE *file;
    char buffer[100];  // 用于存储读取的数据，根据实际情况调整大小

    char fileFolder[100] = "plot_data/";
    char fileFormat[10] = ".txt";

    strcat(fileFolder, fileName);
    strcat(fileFolder, fileFormat);

    // 打开文件
    file = fopen(fileFolder, "r");
    if (file == NULL) {
        perror("无法打开");
    }

    // 读取文件内容
    while (fgets(buffer, sizeof(buffer), file) != NULL) {
        return buffer;
    }

    // 关闭文件
    fclose(file);

}

void removeFile(const char *fileName)
{
    char fileFolder[100] = "plot_data/";
    char fileFormat[10] = ".txt";
    int fileNum;
    strcat(fileFolder, fileName);
    strcat(fileFolder, fileFormat);

    if (remove(fileFolder) == 0) {
        printf("%s 文件删除成功。\n", fileFolder);
    } else {
        perror("删除文件时出错");
    }

}
