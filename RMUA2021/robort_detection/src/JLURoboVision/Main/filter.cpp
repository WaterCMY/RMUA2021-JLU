#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../AngleSolver/filter.h"
#include"../General/General.h"


Filter::Filter()
{
}

Filter::~Filter()
{
}


char kalman_ok = 0;
///*********************************/

kalman_t kalman_x,kalman_y;
kalman_oney kalman_yone,kalman_filter_pitch;

void Filter::kalman2_init(kalman_t* kalman, kalman_oney* kalmany,kalman_oney* kalman_pitch)
{
    if(kalman == &kalman_y)
    {
        //        alloc_matrix_Init(&kalman->test1,3,3);
        //        set_matrix(&kalman->test1, 1.0,2.0,3.0, 1.0,0.0,0.0, 1.0,1.0,1.0);
        //        alloc_matrix_Init(&kalman->test2,3,3);
        //        set_matrix(&kalman->test2, 4.0,3.0,2.0, 1.0,0.0,0.0, 1.0,1.0,1.0);
        //        alloc_matrix_Init(&kalman->end,3,3);
        //        set_matrix(&kalman->end, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);

        //        Matrix2f FTS = Matrix2f::Zero();
        //        Matrix2f FF = Matrix2f::Zero();
        //    Matrix2f TT = Matrix2f::Zero();
        //    Matrix2f ZO = Matrix2f::Zero();

        //    FTS << 1.1,2.2,3.9,4.5;
        //    FF << 1.0,2.0,3.0,4.0;
        //    TT = FTS.inverse();
        //    cout << "TT = !!!.!!!!!!!!" <<endl << TT <<endl;

        alloc_matrix_Init(&kalman->X,3,1);
        set_matrix(&kalman->X,0.0,0.0,0.0);
        // print_matrix(kalman->X);
        alloc_matrix_Init(&kalman->X_mid,3,1);
        set_matrix(&kalman->X_mid, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->X_last,3,1);
        set_matrix(&kalman->X_last, 0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->K,3,3);
        set_matrix(&kalman->K, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->P,3,3);
        set_matrix(&kalman->P, 0.1,0.0,0.0,
                   0.0,0.1,0.0,
                   0.0,0.0,0.1);

        alloc_matrix_Init(&kalman->P_last,3,3);
        set_matrix(&kalman->P_last, 2.0,0.0,0.0,
                   0.0,2.0,0.0,
                   0.0,0.0,2.0);

        alloc_matrix_Init(&kalman->P_mid,3,3);
        set_matrix(&kalman->P_mid, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->Y,3,1);
        set_matrix(&kalman->Y, 0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->F,3,3);
        set_matrix(&kalman->F, 1.0,1.0,0.5,
                   0.0,1.0,1.0,
                   0.0,0.0,1.0);
        alloc_matrix_Init(&kalman->FT,3,3);
        set_matrix(&kalman->FT, 1.0,0.0,0.0,
                   1.0,1.0,0.0,
                   0.5,1.0,1.0);
        alloc_matrix_Init(&kalman->H,3,3);
        set_matrix(&kalman->H, 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0);
        alloc_matrix_Init(&kalman->HT,3,3);
        set_matrix(&kalman->HT, 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0);
        alloc_matrix_Init(&kalman->Q,3,3);
        set_matrix(&kalman->Q, 150.0,0.0,0.0,
                   0.0,30.0,0.0,
                   0.0,0.0,0.8);//系统噪声方差矩阵Q  一般取小值0.0001
        //Q/R-->响应
        //        set_matrix(&kalman->Q, 150.0,0.0,0.0,
        //                               0.0,30.0,0.0,
        //                               0.0,0.0,0.5);//系统噪声方差矩阵Q  一般取小值0.0001
        //                                            //Q/R-->响应
        alloc_matrix_Init(&kalman->R,3,3);
        set_matrix(&kalman->R, 200.0,0.0,0.0,
                   0.0,200.0,0.0,
                   0.0,0.0,20.0);//测量噪声方差矩阵R

        alloc_matrix_Init(&kalman->T,3,3);
        set_matrix(&kalman->T, 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0);

        alloc_matrix_Init(&kalman->buff1_33,3,3);
        set_matrix(&kalman->buff1_33, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff2_33,3,3);
        set_matrix(&kalman->buff2_33, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff3_33,3,3);
        set_matrix(&kalman->buff3_33, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff1_31,3,1);
        set_matrix(&kalman->buff1_31, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff2_31,3,1);
        set_matrix(&kalman->buff2_31, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff3_31,3,1);
        set_matrix(&kalman->buff3_31, 0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->buff1_13,1,3);
        set_matrix(&kalman->buff1_13, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff2_13,1,3);
        set_matrix(&kalman->buff2_13, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff3_13,1,3);
        set_matrix(&kalman->buff3_13, 0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->buff1_11,1,1);
        set_matrix(&kalman->buff1_11, 0.0);
        alloc_matrix_Init(&kalman->buff2_11,1,1);
        set_matrix(&kalman->buff2_11, 0.0);
        alloc_matrix_Init(&kalman->buff3_11,1,1);
        set_matrix(&kalman->buff3_11, 0.0);

        kalmany->X_last = 0.0f;
        kalmany->Pre_last = 0.0f;
        kalmany->q = 1.0f;//0.01f;
        kalmany->r = 10.0f;//change1.0f;
        kalmany->X_mid = 0.0f;


        kalman_pitch->X_last = 0.0f;
        kalman_pitch->Pre_last = 0.0f;
        kalman_pitch->q = 1.0f;//0.01f;
        kalman_pitch->r = 10.0f;//change1.0f;
        kalman_pitch->X_mid = 0.0f;


    }
    else
    {
        alloc_matrix_Init(&kalman->X,3,1);
        set_matrix(&kalman->X,0.0,0.0,0.0);
        // print_matrix(kalman->X);
        alloc_matrix_Init(&kalman->X_mid,3,1);
        set_matrix(&kalman->X_mid, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->X_last,3,1);
        set_matrix(&kalman->X_last, 0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->K,3,3);
        set_matrix(&kalman->K, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->P,3,3);
        set_matrix(&kalman->P, 0.1,0.0,0.0,
                   0.0,0.1,0.0,
                   0.0,0.0,0.1);

        alloc_matrix_Init(&kalman->P_last,3,3);
        set_matrix(&kalman->P_last, 2.0,0.0,0.0,
                   0.0,2.0,0.0,
                   0.0,0.0,2.0);

        alloc_matrix_Init(&kalman->P_mid,3,3);
        set_matrix(&kalman->P_mid, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->Y,3,1);
        set_matrix(&kalman->Y, 0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->F,3,3);
        set_matrix(&kalman->F, 1.0,1.0,0.5,
                   0.0,1.0,1.0,
                   0.0,0.0,1.0);
        alloc_matrix_Init(&kalman->FT,3,3);
        set_matrix(&kalman->FT, 1.0,0.0,0.0,
                   1.0,1.0,0.0,
                   0.5,1.0,1.0);
        alloc_matrix_Init(&kalman->H,3,3);
        set_matrix(&kalman->H, 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0);
        alloc_matrix_Init(&kalman->HT,3,3);
        set_matrix(&kalman->HT, 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0);
        alloc_matrix_Init(&kalman->Q,3,3);
        set_matrix(&kalman->Q, 10.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,0.1);//系统噪声方差矩阵Q  一般取小值0.0001

        alloc_matrix_Init(&kalman->R,3,3);
        set_matrix(&kalman->R, 10.0,0.0,0.0,
                   0.0,10.0,0.0,
                   0.0,0.0,1.0);//测量噪声方差矩阵R

        alloc_matrix_Init(&kalman->T,3,3);
        set_matrix(&kalman->T, 1.0,0.0,0.0,
                   0.0,1.0,0.0,
                   0.0,0.0,1.0);

        alloc_matrix_Init(&kalman->buff1_33,3,3);
        set_matrix(&kalman->buff1_33, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff2_33,3,3);
        set_matrix(&kalman->buff2_33, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff3_33,3,3);
        set_matrix(&kalman->buff3_33, 0.0,0.0,0.0,
                   0.0,0.0,0.0,
                   0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff1_31,3,1);
        set_matrix(&kalman->buff1_31, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff2_31,3,1);
        set_matrix(&kalman->buff2_31, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff3_31,3,1);
        set_matrix(&kalman->buff3_31, 0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->buff1_13,1,3);
        set_matrix(&kalman->buff1_13, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff2_13,1,3);
        set_matrix(&kalman->buff2_13, 0.0,0.0,0.0);
        alloc_matrix_Init(&kalman->buff3_13,1,3);
        set_matrix(&kalman->buff3_13, 0.0,0.0,0.0);

        alloc_matrix_Init(&kalman->buff1_11,1,1);
        set_matrix(&kalman->buff1_11, 0.0);
        alloc_matrix_Init(&kalman->buff2_11,1,1);
        set_matrix(&kalman->buff2_11, 0.0);
        alloc_matrix_Init(&kalman->buff3_11,1,1);
        set_matrix(&kalman->buff3_11, 0.0);
    }
    kalman_ok =1;
}

float Filter::kalman_CalY(kalman_oney* kalman_y, double dat)
{
    kalman_y->X_mid = kalman_y->X_last;
    kalman_y->Pre_mid = kalman_y->Pre_last + kalman_y->q;
    kalman_y->kg = kalman_y->Pre_mid / (kalman_y->Pre_mid + kalman_y->r);
    kalman_y->X_now = kalman_y->X_mid + kalman_y->kg * (dat- kalman_y->X_mid);
    kalman_y->Pre_now = (1 - kalman_y->kg ) * kalman_y->Pre_mid;
    kalman_y->Pre_last = kalman_y->Pre_now;
    kalman_y->X_last = kalman_y->X_now;

    //    *filter_x = (float)kalman->X.data[0][0];
    //    *filter_v = (float)kalman->X.data[1][0];
    //    *filter_a = (float)kalman->X.data[2][0];

    return kalman_y->X_now;
}



float* Filter::kalman_Cal(kalman_t* kalman, double p, double v, double a, double * filter_x , double * filter_v, double * filter_a)
{
    if(kalman_ok!=1)
        return NULL;

    //    kalman->X_last = kalman->X;
    //    kalman->P_last = kalman->P;
    //    memcpy(&kalman->X_last.data[0][0],&kalman->X.data[0][0],8);
    //    memcpy(&kalman->P_last.data[0][0],&kalman->P.data[0][0],16);
    // print_matrix(kalman->X);


    copy_matrix(kalman->X,kalman->X_last);
    copy_matrix(kalman->P,kalman->P_last);
    //    print_matrix(kalman->X);

    //test
    //destructive_invert_matrix(kalman->test1, kalman->test2);
    //    multiply_matrix((kalman->test1),(kalman->test2),kalman->end);
    //    print_matrix(kalman->end);

    multiply_matrix((kalman->F),(kalman->X_last),kalman->X_mid);
    // print_matrix(kalman->X_mid);

    multiply_matrix((kalman->F),(kalman->P_last),kalman->buff1_33);

    multiply_matrix((kalman->buff1_33),(kalman->FT),kalman->buff3_33);

    add_matrix((kalman->buff3_33),(kalman->Q),kalman->P_mid);

    multiply_matrix((kalman->H),(kalman->P_mid),kalman->buff3_33);
    multiply_matrix((kalman->buff3_33),(kalman->HT),kalman->buff2_33);

    add_matrix((kalman->buff2_33),(kalman->R),kalman->buff1_33);
    destructive_invert_matrix(kalman->buff1_33,kalman->buff3_33);

    multiply_matrix((kalman->P_mid),(kalman->HT),kalman->buff2_33);
    multiply_matrix((kalman->buff2_33),(kalman->buff3_33),kalman->K);

    set_matrix(&kalman->Y,p,v,a);

    //print_matrix(kalman->Y);
    multiply_matrix((kalman->H),(kalman->X_mid),kalman->buff1_31);
    subtract_matrix((kalman->Y),(kalman->buff1_31),kalman->buff2_31);
    multiply_matrix((kalman->K),(kalman->buff2_31),kalman->buff3_31);

    add_matrix((kalman->X_mid),(kalman->buff3_31),kalman->X);

    multiply_matrix((kalman->K),(kalman->H),kalman->buff1_33);
    multiply_matrix((kalman->T),(kalman->buff1_33),kalman->buff2_33);

    multiply_matrix((kalman->buff2_33),(kalman->P_mid),kalman->P);

    *filter_x = (float)kalman->X.data[0][0];
    *filter_v = (float)kalman->X.data[1][0];
    *filter_a = (float)kalman->X.data[2][0];

    //    printf(" filter_p : %f\n", *filter_x);
    //    printf(" filter_v : %f\n", *filter_v);
    //    printf(" filter_a : %f\n", *filter_a);

    //    printf("print->X\n");
    //    print_matrix(kalman->X);

    return NULL;
}

///* Matrix math. */


//#include "matrix.h"


/* 为矩阵分配初始空间 */
Matrix_Other alloc_matrix_Init(Matrix_Other* m,int rows, int cols) {
    int i,j;
    m->rows = rows;
    m->cols = cols;
    m->data = (double**) malloc(sizeof(double*) * m->rows);

    for ( i = 0; i < m->rows; ++i)
    {
        m->data[i] = (double*) malloc(sizeof(double) * m->cols);
        assert(m->data[i]);
        for ( j = 0; j < m->cols; ++j) {
            m->data[i][j] = 0.0;
        }
    }
    //  return m;
}
/* 释放空间 */
void free_matrix(Matrix_Other m) {
    int i;
    assert(m.data != NULL);
    for ( i = 0; i < m.rows; ++i) {
        free(m.data[i]);
    }
    free(m.data);
}
///* 初始化矩阵 */
void set_matrix(Matrix_Other* m, ...) {
    va_list ap;
    int i,j;
    va_start(ap, m);

    for ( i = 0; i < m->rows; ++i) {
        for ( j = 0; j < m->cols; ++j) {
            m->data[i][j] = va_arg(ap, double);
        }
    }

    va_end(ap);
}
/* 转换为单元矩阵 */
void set_identity_matrix(Matrix_Other m) {
    int i;
    int j;
    assert(m.rows == m.cols);
    for ( i = 0; i < m.rows; ++i) {
        for ( j = 0; j < m.cols; ++j) {
            if (i == j) {
                m.data[i][j] = 1.0;
            } else {
                m.data[i][j] = 0.0;
            }
        }
    }
}
/* 复制矩阵 */
void copy_matrix(Matrix_Other source, Matrix_Other destination) {
    int i;
    int j;
    assert(source.rows == destination.rows);
    assert(source.cols == destination.cols);
    for ( i = 0; i < source.rows; ++i) {
        for ( j = 0; j < source.cols; ++j) {
            destination.data[i][j] = source.data[i][j];
        }
    }
}
/* 打印矩阵 */
void print_matrix(Matrix_Other m) {
    int i;
    int j;
    for ( i = 0; i < m.rows; ++i) {
        for ( j = 0; j < m.cols; ++j) {
            if (j > 0) {
                printf(" ");
            }
            printf("%6.6f", m.data[i][j]);
        }
        printf("\n");
    }
}
/* 矩阵相加 */
void add_matrix(Matrix_Other a, Matrix_Other b, Matrix_Other c) {
    int i;
    int j;
    assert(a.rows == b.rows);
    assert(a.rows == c.rows);
    assert(a.cols == b.cols);
    assert(a.cols == c.cols);
    for ( i = 0; i < a.rows; ++i) {
        for ( j = 0; j < a.cols; ++j) {
            c.data[i][j] = a.data[i][j] + b.data[i][j];
        }
    }
}
/* 矩阵相减 */
void subtract_matrix(Matrix_Other a, Matrix_Other b, Matrix_Other c) {
    int i;
    int j;
    assert(a.rows == b.rows);
    assert(a.rows == c.rows);
    assert(a.cols == b.cols);
    assert(a.cols == c.cols);
    for ( i = 0; i < a.rows; ++i) {
        for ( j = 0; j < a.cols; ++j) {
            c.data[i][j] = a.data[i][j] - b.data[i][j];
        }
    }
}
/* 用单元矩阵减去该矩阵 */
void subtract_from_identity_matrix(Matrix_Other a) {
    int i;
    int j;
    assert(a.rows == a.cols);
    for ( i = 0; i < a.rows; ++i) {
        for ( j = 0; j < a.cols; ++j) {
            if (i == j) {
                a.data[i][j] = 1.0 - a.data[i][j];
            } else {
                a.data[i][j] = 0.0 - a.data[i][j];
            }
        }
    }
}
/* 矩阵相乘 */
void multiply_matrix(Matrix_Other a, Matrix_Other b, Matrix_Other c) {
    int i;
    int j;
    int k;
    assert(a.cols == b.rows);
    assert(a.rows == c.rows);
    assert(b.cols == c.cols);
    for ( i = 0; i < c.rows; ++i) {
        for ( j = 0; j < c.cols; ++j) {
            /* Calculate element c.data[i][j] via a dot product of one row of a
     with one column of b */
            c.data[i][j] = 0.0;
            for ( k = 0; k < a.cols; ++k) {
                c.data[i][j] += a.data[i][k] * b.data[k][j];
            }
        }
    }
}

/* 乘以一个矩阵的转置矩阵. */
void multiply_by_transpose_matrix(Matrix_Other a, Matrix_Other b, Matrix_Other c) {
    int i;
    int j;
    int k;
    assert(a.cols == b.cols);
    assert(a.rows == c.rows);
    assert(b.rows == c.cols);
    for ( i = 0; i < c.rows; ++i) {
        for ( j = 0; j < c.cols; ++j) {
            /* Calculate element c.data[i][j] via a dot product of one row of a
     with one row of b */
            c.data[i][j] = 0.0;
            for ( k = 0; k < a.cols; ++k) {
                c.data[i][j] += a.data[i][k] * b.data[j][k];
            }
        }
    }
}
/* 矩阵转置 */
void transpose_matrix(Matrix_Other input, Matrix_Other output) {
    int i;
    int j;
    //int k;
    assert(input.rows == output.cols);
    assert(input.cols == output.rows);
    for ( i = 0; i < input.rows; ++i) {
        for ( j = 0; j < input.cols; ++j) {
            output.data[j][i] = input.data[i][j];
        }
    }
}
/* 两矩阵是否相等 */
int equal_matrix(Matrix_Other a, Matrix_Other b, double tolerance) {
    int i;
    int j;
    //int k;
    assert(a.rows == b.rows);
    assert(a.cols == b.cols);
    for ( i = 0; i < a.rows; ++i) {
        for ( j = 0; j < a.cols; ++j) {
            if (abs(a.data[i][j] - b.data[i][j]) > tolerance) {
                return 0;
            }
        }
    }
    return 1;
}
/* 矩阵乘以一个系数 */
void scale_matrix(Matrix_Other m, double scalar) {
    int i;
    int j;

    assert(scalar != 0.0);
    for ( i = 0; i < m.rows; ++i) {
        for ( j = 0; j < m.cols; ++j) {
            m.data[i][j] *= scalar;
        }
    }
}
/* 交换矩阵的两行 */
void swap_rows(Matrix_Other m, int r1, int r2) {
    double *tmp;
    assert(r1 != r2);
    tmp = m.data[r1];
    m.data[r1] = m.data[r2];
    m.data[r2] = tmp;
}
/* 矩阵某行乘以一个系数 */
void scale_row(Matrix_Other m, int r, double scalar) {
    int i;
    assert(scalar != 0.0);
    for ( i = 0; i < m.cols; ++i) {
        m.data[r][i] *= scalar;
    }
}

/* Add scalar * row r2 to row r1. */
void shear_row(Matrix_Other m, int r1, int r2, double scalar) {
    int i;
    assert(r1 != r2);
    for ( i = 0; i < m.cols; ++i) {
        m.data[r1][i] += scalar * m.data[r2][i];
    }
}

/* 矩阵的求逆(借鉴他人) */
/* Uses Gauss-Jordan elimination.

   The elimination procedure works by applying elementary row
   operations to our input matrix until the input matrix is reduced to
   the identity matrix.
   Simultaneously, we apply the same elementary row operations to a
   separate identity matrix to produce the inverse matrix.
   If this makes no sense, read wikipedia on Gauss-Jordan elimination.

   This is not the fastest way to invert matrices, so this is quite
   possibly the bottleneck. */
int destructive_invert_matrix(Matrix_Other input, Matrix_Other output) {
    int i;
    int j;
    int r;
    double scalar;
    double shear_needed;
    assert(input.rows == input.cols);
    assert(input.rows == output.rows);
    assert(input.rows == output.cols);

    set_identity_matrix(output);

    /* Convert input to the identity matrix via elementary row operations.
     The ith pass through this loop turns the element at i,i to a 1
     and turns all other elements in column i to a 0. */

    for ( i = 0; i < input.rows; ++i) {

        if (input.data[i][i] == 0.0) {
            /* We must swap rows to get a nonzero diagonal element. */

            for (r = i + 1; r < input.rows; ++r) {
                if (input.data[r][i] != 0.0) {
                    break;
                }
            }
            if (r == input.rows) {
                /* Every remaining element in this column is zero, so this
       matrix cannot be inverted. */
                return 0;
            }
            swap_rows(input, i, r);
            swap_rows(output, i, r);
        }

        /* Scale this row to ensure a 1 along the diagonal.
       We might need to worry about overflow from a huge scalar here. */
        scalar = 1.0 / input.data[i][i];
        scale_row(input, i, scalar);
        scale_row(output, i, scalar);

        /* Zero out the other elements in this column. */
        for ( j = 0; j < input.rows; ++j) {
            if (i == j) {
                continue;
            }
            shear_needed = -input.data[j][i];
            shear_row(input, j, i, shear_needed);
            shear_row(output, j, i, shear_needed);
        }
    }

    return 1;
}
