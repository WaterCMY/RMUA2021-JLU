//#ifndef PREDICT_H
//#define PREDICT_H

//#include     "../Serial/Serial.h"
//#include "../AngleSolver/filter.h"
//class predict

//{
//public:
//    predict();
//};

//typedef struct enemy
//{
//    double p;
//    double p_filter;
//    double last_p;
//    double y;
//    double y_filter;
//    double last_y;

//    double vp;
//    double vp_filter;
//    double vp_last;
//    double vy;
//    double vy_filter;
//    double vy_last;

//    double ap;
//    double ap_filter;
//    double ay;
//    double ay_filter;

//    float last_vp;
//    float last_vy;

//    double Angle_Pitch;
//    double Angle_yaw;
//    double H;
//    double gravity_pitch;

//    float distance;
//    float horizontal_dis;
//    float angle360;
//    float last_angle;
//    float w_speed;
//    int id;
//    uint cnt;

//    uint32_t last_time;
//    uint32_t now_time;
//}enemy_t;

////获取预判的虚拟敌人
//enemy_t* Enemy_PosForecast(enemy_t* enemy, int armor_change, Filter& filter_);

////攻击敌人位置
//enemy_t* Enemy_CalNow(double X,double Y, double Z, vision_pr* attitude, Filter& filter_);

////dragon
//void vision_to_embedded(double X, double Y ,double Z, bool find ,int armor_change, vision_pr* attitude, double* pitch, double* yaw, Filter& filter_, vision_te* test);


////接口
//float Vision_GravityCompensation(float distance,float high,float speed);//重力补偿
//float Vision_HighCompensation(vision_pr* attitude);//high补偿

//#endif // PREDICT_H




#ifndef PREDICT_H
#define PREDICT_H

#include     "../Serial/Serial.h"
#include "../AngleSolver/filter.h"
class predict

{
public:
    predict();
};

typedef struct enemy
{
    double x;
    double x_filter;
    double last_x;
    double y;
    double y_filter;
    double last_y;

    double Z;

    double vx;
    double vx_filter;
    double vx_last;
    double vy;
    double vy_filter;
    double vy_last;

    double ax;
    double ax_filter;
    double ay;
    double ay_filter;

    float last_vx;
    float last_vy;

    float distance;
    float horizontal_dis;
    float angle360;
    float last_angle;
    float w_speed;
    int id;
    uint cnt;

    uint32_t last_time;
    uint32_t now_time;
}enemy_t;

//获取预判的虚拟敌人
enemy_t* Enemy_PosForecast(enemy_t* enemy,float shoot_speed, int armor_change, Filter& filter_, bool find);

//攻击敌人位置
enemy_t* Enemy_CalNow(double X,double Y, double Z, vision_pr* attitude, Filter& filter_);

//dragon
void vision_to_embedded(double X, double Y ,double Z, bool find ,int armor_change, vision_pr* attitude, double* pitch, double* yaw, Filter& filter_, vision_te* test);

float Hori_Distance(enemy_t* enemy,vision_pr* attitude,float high_comp);
//接口
float Vision_GravityCompensation(float distance,float high,float speed);//重力补偿
float Vision_HighCompensation(vision_pr* attitude , float enemy_Z);//high补偿

#endif // PREDICT_H
