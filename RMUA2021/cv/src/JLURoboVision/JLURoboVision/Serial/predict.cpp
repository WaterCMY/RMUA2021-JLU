#include "../AngleSolver/filter.h"
#include "../General/General.h"
#include "predict.h"
#include "Serial.h"


//#define HIGH  0.88started core service [/rosout]

#define SHOOTERLENTH 0.06
//计算敌人当前位置

enemy_t*  Enemy_CalNow(double X,double Y, double Z, vision_pr* attitude, Filter& filter_)
{

    static enemy_t enemy;
    if(attitude->yaw_now < 0.0){
        //attitude->yaw_now += 360.0;
    }

    enemy.y = Z*cos((-attitude->pitch_now *3.1415)/180.0) - Y*sin((-attitude->pitch_now *3.1415)/180.0);
    enemy.x = X;
    enemy.Z = Z*cos(((90 + attitude->pitch_now) *3.1415)/180.0) + Y*sin(((90 + attitude->pitch_now) *3.1415)/180.0);

    enemy.horizontal_dis = sqrt(enemy.x*enemy.x+enemy.y*enemy.y);
    enemy.angle360 = attitude->yaw_now - (asin(enemy.x/enemy.horizontal_dis)*180.0)/3.14159;


    enemy.x = enemy.horizontal_dis * cos(enemy.angle360*3.1415926/180.0);
    enemy.y = enemy.horizontal_dis * sin(enemy.angle360*3.1415926/180.0);
    enemy.distance = sqrt(enemy.horizontal_dis*enemy.horizontal_dis + enemy.Z*enemy.Z);




    enemy.last_y = enemy.y_filter;
    enemy.last_x = enemy.x_filter;
//	enemy->last_angle = angle_filter ;

    enemy.last_vx = enemy.vx_filter;
    enemy.last_vy = enemy.vy_filter;

    enemy.y = filter_.kalman_CalY(&kalman_yone , enemy.y);
    filter_.kalman_Cal(&kalman_y, enemy.y, enemy.vy, enemy.ay , &enemy.y_filter, &enemy.vy_filter, &enemy.ay_filter);
    filter_.kalman_Cal(&kalman_x, enemy.x, enemy.vx, enemy.ax, &enemy.x_filter, &enemy.vx_filter, &enemy.ax_filter);



    enemy.vy = (enemy.y_filter - enemy.last_y);
    enemy.vx = (enemy.x_filter - enemy.last_x);

    enemy.ay = (enemy.vy_filter - enemy.last_vy);
    enemy.ax = (enemy.vx_filter - enemy.last_vx);

    return &enemy;
}
//计算敌人并预判
enemy_t* Enemy_PosForecast(enemy_t* enemy,float shoot_speed, int armor_change, Filter& filter_, bool find)
{
   //return enemy;
    static int find_num = 0;
    if(find == 1){
        find_num +=1;
    }
   if (armor_change == 3 || find_num <= 15)
   {
       return enemy;
   }

    static enemy_t predict_enemy;
    static int lost_num = 0;
    if((armor_change != 2 )&& armor_change != 1){
        lost_num += 1;
    }
    //return enemy;
    if(   lost_num >= 15 ){		//前15帧no predict
        find_num = 0;
        double px, vx, ax, py, vy, ay;
        float X, pitch_filter;
        X = kalman_yone.X_now;
        pitch_filter = kalman_filter_pitch.X_now;
        px = kalman_x.X.data[0][0];
        vx = kalman_x.X.data[1][0];
        ax = kalman_x.X.data[2][0];
        py = kalman_y.X.data[0][0];
        vy = kalman_y.X.data[1][0];
        ay = kalman_y.X.data[2][0];
        filter_.kalman2_init(&kalman_x, &kalman_yone, &kalman_filter_pitch);
        filter_.kalman2_init(&kalman_y, &kalman_yone, &kalman_filter_pitch);
        kalman_yone.X_last = X;
        kalman_filter_pitch.X_last;
        set_matrix(&kalman_x.X,px,vx,ax);
        set_matrix(&kalman_y.X,py,vy,ay);
        lost_num = 0;
    }

    if(shoot_speed==0)
        shoot_speed = 20;

    float mech_delay, shoot_delay, t;
    shoot_delay = ( enemy->distance / shoot_speed ) * 28.0;
    mech_delay = 3.0;
    t = shoot_delay + mech_delay;

    if(1)//bu li pu
    {
        predict_enemy.x = enemy->x_filter + enemy->vx_filter*t + 0.5*enemy->ax_filter*t*t;
        predict_enemy.y = enemy->y_filter + enemy->vy_filter*t + 0.5*enemy->ay_filter*t*t;
        predict_enemy.Z = enemy->Z;
        predict_enemy.horizontal_dis = sqrt(predict_enemy.x*predict_enemy.x+predict_enemy.y*predict_enemy.y);
        predict_enemy.distance = sqrt(predict_enemy.horizontal_dis*predict_enemy.horizontal_dis + enemy->Z*enemy->Z);

        //printf("pre_x : %f\t", predict_enemy.x);
        //printf("pre_y : %f\n", predict_enemy.y);
        //printf("pre_Z : %f\n", enemy->Z);

        if(predict_enemy.y > 0.0){
            predict_enemy.angle360=(acos(predict_enemy.x/predict_enemy.horizontal_dis)*180.0)/3.14159;//-1.5;
        }else{
            predict_enemy.angle360= -(acos(predict_enemy.x/predict_enemy.horizontal_dis)*180.0)/3.14159;//-1.5;
            if(predict_enemy.angle360 < -180.0 ){
                //predict_enemy.angle360 = 360.0 + predict_enemy.angle360;
            }
        }
    }

//    printf("predict_enemy.distance : %f  \n", predict_enemy.distance);
    //距离在attack中计算
    return &predict_enemy;
}



void vision_to_embedded(double X, double Y ,double Z, bool find,int armor_change, vision_pr* attitude, double* pitch, double* yaw, Filter& filter_, vision_te* test)
{
    if(find == 1)
    {
	//X +=50.0;
        X = X * 0.001 ;
        Y = Y * 0.001 ;//视觉传输为360度乘以100的数据
        Z = (Z+SHOOTERLENTH*1000) * 0.001 ;
        enemy_t* save_enemy = Enemy_CalNow(X, Y, Z, attitude, filter_);

        //printf("save_enemy->x : %f \t", save_enemy->x);
        //printf("save_enemy->vy_filter : %f\n", save_enemy->vy_filter);

        enemy_t* end_pr_enemy = Enemy_PosForecast(save_enemy, attitude->Bullet_speed, armor_change, filter_, find);

        float high_comp,hori_distance_comp;
        high_comp = Vision_HighCompensation(attitude, save_enemy->Z);

//        *pitch = save_enemy->x;
        high_comp = 0.28;
//        end_pr_enemy->distance = 10;
        hori_distance_comp = Hori_Distance(end_pr_enemy , attitude , high_comp);
//        hori_distance_comp =3.50;
        *pitch = Vision_GravityCompensation(hori_distance_comp, high_comp, attitude->Bullet_speed);
        *pitch = filter_.kalman_CalY(&kalman_filter_pitch , *pitch);

//        *pitch = -(asin(save_enemy->Z/save_enemy->horizontal_dis)*180.0)/3.14159;

//        if (*pitch>17)
//        {
//            *pitch = (acos(save_enemy->y/save_enemy->distance))*180.0/3.1415;
//        }
//        *yaw = save_enemy->y;
        *yaw = end_pr_enemy->angle360-2.0;  
//        *pitch =  (*pitch/3.1415926)*180.0;
//        *yaw = (*yaw/3.1415926)*180.0;


        //**************vision test****************//
        test->enemy_angle_360 = save_enemy->angle360;
        test->x_filter = save_enemy->x_filter;
        test->vx_filter = save_enemy->vx_filter;
        test->ax_filter = save_enemy->ax_filter;
        test->y_filter = save_enemy->y_filter;
        test->vy_filter = save_enemy->vy_filter;
        test->ay_filter = save_enemy->ay_filter;
        test->pre_yaw = *yaw;
        test->pre_pitch = *pitch;
    }

}
float Hori_Distance(enemy_t* enemy,vision_pr* attitude,float high_comp)
{



    float distance = (enemy->horizontal_dis - (SHOOTERLENTH*cos((attitude->pitch_now)*3.1415/180.0)));
    distance = sqrt(distance*distance + high_comp*high_comp);
    return distance;
}
float Vision_GravityCompensation(float distance,float high,float speed)//重力补偿
{

    float t,derta,v_x,pitch, v_y;
    float speed_h;
    speed_h = 15;
    if(speed!=0){
        speed_h = speed-3.0;}
    derta =(high*9.8f+speed_h*speed_h)*(high*9.8f+speed_h*speed_h)-9.8f*9.8f*distance*distance;
    t = ((high*9.8f+speed_h*speed_h)-sqrt(derta))/(0.5f*9.8f*9.8f);
    v_x = sqrt((distance*distance-(high*high))/t);
    v_y = sqrt(speed*speed - v_x*v_x);

    if(v_x>speed_h)
        v_x = speed_h;
    if(high < 0 ){
        pitch = (acos(v_x/speed_h)*180.0)/3.14159;
    }else{
        if(t>(2*high/9.8f)){
            pitch =(acos(v_x/speed_h)*180.0)/3.14159;
        }else{
            pitch =-(acos(v_x/speed_h)*180.0)/3.14159;
        }
    }

    return pitch;
}

float Vision_HighCompensation(vision_pr* attitude, float enemy_Z)//high补偿
{
    float high;
    high = enemy_Z - sin(-attitude->pitch_now*2*3.14159/360.0)*SHOOTERLENTH;
    return high;
}
