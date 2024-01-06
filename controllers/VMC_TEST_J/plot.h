#ifndef PLOT_H
#define PLOT_H



void plotFile(const char *fileName, float input_x, float input_y);
void plotFile3(const char *fileName, float input_t, float input_x, float input_y, float input_z);
void plotFile6(const char *fileName, float input_t, float input_x, float input_y, float input_z, float input_a, float input_b, float input_c);
void numFile(const char *fileName, float input_num);
int readNum(const char *fileName);
void removeFile(const char *fileName);
void plotInit();

#endif
