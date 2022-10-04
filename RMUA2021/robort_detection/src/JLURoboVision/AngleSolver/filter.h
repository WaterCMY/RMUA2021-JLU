#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"
#include "../General/General.h"

typedef struct {
    /* 维度 */
    int rows;
    int cols;
    /* 内容 */
    double** data;
} Matrix_Other;


typedef struct {
    Matrix_Other H;
    Matrix_Other HT;
    Matrix_Other F;

    Matrix_Other FT;
    Matrix_Other P;
    Matrix_Other P_last;
    Matrix_Other P_mid;
    Matrix_Other Q;
    Matrix_Other R;
    Matrix_Other X;

    Matrix_Other X_last;
    Matrix_Other X_mid;
    Matrix_Other Y;//guan ce zhi
    Matrix_Other T;
    Matrix_Other K;

    Matrix_Other test1;
    Matrix_Other test2;
    Matrix_Other end;

    Matrix_Other buff1_33;
    Matrix_Other buff2_33;
    Matrix_Other buff3_33;

    Matrix_Other buff1_13;
    Matrix_Other buff2_13;
    Matrix_Other buff3_13;

    Matrix_Other buff1_31;
    Matrix_Other buff2_31;
    Matrix_Other buff3_31;

    Matrix_Other buff1_11;
    Matrix_Other buff2_11;
    Matrix_Other buff3_11;

}kalman_t;

typedef struct {
    float X_last;
    float X_mid;
    float X_now;
    float Pre_mid;
    float Pre_last;//当前时刻最优结果的协方差
    float Pre_now;//上一次最优结果的协方差
    float kg;//增益
    float A;//系统参数
    float B;
    float q;
    float r;
    float h;

}kalman_oney;

//Matrix2f two_matrix_cross(Matrix2f end, Matrix2f one,Matrix2f two);

class Filter
{
public:
    Filter();
    ~Filter();

    float kalman_CalY(kalman_oney* kalman, double Y);

    float* kalman_Cal(kalman_t* kalman,double p, double v, double a, double * filter_x , double* filter_v ,double * filter_a);
    void kalman2_init(kalman_t* kalman, kalman_oney* kalmany, kalman_oney* kalman_filter_pitch);
};


extern kalman_t kalman_x,kalman_y;
extern kalman_oney kalman_yone,kalman_filter_pitch;



Matrix_Other alloc_matrix_Init(Matrix_Other* m,int rows, int cols);
void set_matrix(Matrix_Other* m, ...);
void add_matrix(Matrix_Other a, Matrix_Other b, Matrix_Other c);
void subtract_matrix(Matrix_Other a, Matrix_Other b, Matrix_Other c);
void multiply_matrix(Matrix_Other a, Matrix_Other b, Matrix_Other c);
int destructive_invert_matrix(Matrix_Other input, Matrix_Other output);
void copy_matrix(Matrix_Other source, Matrix_Other destination);
void print_matrix(Matrix_Other m);
#endif // PREDICT_H




