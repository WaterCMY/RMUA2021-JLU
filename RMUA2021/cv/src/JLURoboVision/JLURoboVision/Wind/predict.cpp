#include "Energy.h"

void WindDetector::getTargetPolarAngle() {
    if(OpenBigWind) target_polar_angle = static_cast<float>(atan2((-1 * (targetArmors[0].center.y - circle_center_point.y)), (targetArmors[0].center.x - circle_center_point.x)));
    else target_polar_angle = static_cast<float>(180 / 3.14 * atan2((-1 * (targetArmors[0].center.y - circle_center_point.y)), (targetArmors[0].center.x - circle_center_point.x)));//弧度转角度
    if (OpenBigWind) {
        Time[NowTime] = static_cast<double>(getTickCount());
        Angle[NowTime] = target_polar_angle;
        //printf("time:%.3lf angle:%f\n", (double)(Time[NowTime]) / CLOCKS_PER_SEC, Angle[NowTime]);//(double)(Time[NowTime] / CLOCKS_PER_SEC)
        if (!FirstTime) {
            int PreTime = (NowTime - 1) == -1 ? 2 : (NowTime - 1);
            ChangeTime = (Time[NowTime] - Time[PreTime]) / getTickFrequency();//两帧相差时间
            //printf("changetime: ccccccc  %.3lf\n",ChangeTime);
            if (ChangeTime > 0.3) {//间隔时间过长，认为丢失目标，进行重置
                //printf("chongzhi               time:%.3lf\n", ChangeTime);
                for (int i = 0; i < 3; i++) {
                    Time[i] = 0;
                    Angle[i] = 0;
                }
                FirstTime = true;
                NowTime = 0; NowSpeed = 0;
                Speed[0] = 0; Speed[1] = 0;
                NumSp = 0; NowWindTime = 0;
                acc = 0;
                firstPredict = true;
                PredictAngle = 0;
            }
            else {
                //printf("pre:%.3lf now:%.3lf changeangle:%.3lf\n",Angle[PreTime],Angle[NowTime],abs((Angle[NowTime] - Angle[PreTime])));
                Speed[NowSpeed] = abs((Angle[NowTime] - Angle[PreTime])) / ChangeTime;//弧度速度
                if(Speed[NowSpeed]>10) Speed[NowSpeed] = abs((Angle[NowTime] + Angle[PreTime])) / ChangeTime;
                NumSp++;
                //printf("time:%5.3lf sssssssss speed:%5.3lf\n", (double)(Time[NowTime]) / CLOCKS_PER_SEC, Speed[NowSpeed]);
                if (NumSp > 1) {
                    double ChangeSpeed = Speed[NowSpeed] - Speed[NowSpeed == 1 ? 0 : 1];
                    acc = ChangeSpeed / ChangeTime;
                    double x = (Speed[NowSpeed] - 1.305) / 0.785;
                    if (x > 1) NowWindTime = asin(1) / 1.884;
                    else if (x < -1) NowWindTime = asin(-1) / 1.884;
                    else NowWindTime = asin(x) / 1.884;
                    if (ChangeSpeed < 0) {
                        if (NowWindTime > 0) NowWindTime = 1.667 - NowWindTime;
                        else NowWindTime = -1.667 - NowWindTime;
                    }
                    //printf("change:%.3lf nowtime:zzzzzzzzzzzzz%.3lf\n", x,NowWindTime);
                }
                NowSpeed = (NowSpeed + 1) % 2;
            }
        }
        else FirstTime = false;
        NowTime = (NowTime + 1) % 3;
    }
}

void WindDetector::initRotation() {
    if (target_polar_angle >= -180 && last_target_polar_angle_judge_rotation >= -180
        && fabs(target_polar_angle - last_target_polar_angle_judge_rotation) < 30) {
        //target_polar_angle和last_target_polar_angle_judge_rotation的初值均为1000，大于-180表示刚开始几帧不要
        //若两者比较接近，则说明没有切换目标，因此可以用于顺逆时针的判断
        if (target_polar_angle < last_target_polar_angle_judge_rotation) clockwise_rotation_init_cnt++;
        else if (target_polar_angle > last_target_polar_angle_judge_rotation) anticlockwise_rotation_init_cnt++;
    }
    //由于刚开始圆心判断不准，角度变化可能计算有误，因此需要在角度正向或逆向变化足够大时才可确定是否为顺逆时针
    if (clockwise_rotation_init_cnt == 30) {
        energy_rotation_direction = 1;//顺时针变化15次，确定为顺时针
        cout << "rotation: " << energy_rotation_direction << endl;
        energy_rotation_init = false;
    }
    else if (anticlockwise_rotation_init_cnt == 30) {
        energy_rotation_direction = -1;//逆时针变化15次，确定为逆时针
        cout << "rotation: " << energy_rotation_direction << endl;
        energy_rotation_init = false;
    }
    last_target_polar_angle_judge_rotation = target_polar_angle;
}

void WindDetector::getPredictPoint(cv::Point target_point) {
    if (OpenBigWind) {
        if (energy_rotation_direction == 1 && NumSp > 1) predict_rad = CalPreRad();
        else if (energy_rotation_direction == -1 && NumSp > 1)predict_rad = CalPreRad() * (-1);
    }
    else {
        if (energy_rotation_direction == 1) predict_rad = predict_rad_norm;
        else if (energy_rotation_direction == -1) predict_rad = -predict_rad_norm;
    }
    rotate(target_point);

}

void WindDetector::rotate(cv::Point target_point) {
    int x1, x2, y1, y2;
    //    为了减小强制转换的误差
    x1 = circle_center_point.x * 100;
    x2 = target_point.x * 100;
    y1 = circle_center_point.y * 100;
    y2 = target_point.y * 100;
    if (OpenBigWind) {
        double pre_angle = 0,next_angle = 0;
        Point prePoint;
        prePoint = Predict_point;
        pre_angle = static_cast<float>(atan2((-1 * (Predict_point.y - circle_center_point.y)), (Predict_point.x - circle_center_point.x)));
        Predict_point.x = static_cast<int>(
            (x1 + (x2 - x1) * cos(-predict_rad) - (y1 - y2) * sin(-predict_rad)) / 100);
        Predict_point.y = static_cast<int>(
            (y1 - (x2 - x1) * sin(-predict_rad) - (y1 - y2) * cos(-predict_rad)) / 100);
        Predict_point.x=kalman1_filter_calc(PreX,Predict_point.x);
        Predict_point.y=kalman1_filter_calc(PreY,Predict_point.y);
        next_angle = static_cast<float>(atan2((-1 * (Predict_point.y - circle_center_point.y)), (Predict_point.x - circle_center_point.x)));
        PredictAngle = next_angle;
        //printf("pre      :%.6lf  next    :%.6lf\n",pre_angle,next_angle);
        if((!firstPredict)&&energy_rotation_direction==1){
            if(pre_angle<0&&next_angle>0) return ;
            if(next_angle>pre_angle){
                /*
                double predict_rad1 = ChangeTime * Speed[NowSpeed];//(Speed[0] + Speed[1]) / 2
                Predict_point.x = static_cast<int>((x1 + (prePoint.x - x1) * cos(-predict_rad1) - (y1 - prePoint.y) * sin(-predict_rad1)) / 100);
                Predict_point.y = static_cast<int>((y1 - (prePoint.x - x1) * sin(-predict_rad1) - (y1 - prePoint.y) * cos(-predict_rad1)) / 100);
                */
                Predict_point = prePoint;
                PredictAngle = pre_angle;
            }
        }
        else if((!firstPredict)&&energy_rotation_direction==-1){
            if(next_angle<0&&pre_angle>0) return ;
            if(next_angle<pre_angle){
                /*
                double predict_rad1 = (-1) * ChangeTime * Speed[NowSpeed];//(Speed[0] + Speed[1]) / 2
                Predict_point.x = static_cast<int>(
                    (x1 + (prePoint.x - x1) * cos(-predict_rad1) - (y1 - prePoint.y) * sin(-predict_rad1)) / 100);
                Predict_point.y = static_cast<int>(
                    (y1 - (prePoint.x - x1) * sin(-predict_rad1) - (y1 - prePoint.y) * cos(-predict_rad1)) / 100);
                */
                Predict_point = prePoint;
                PredictAngle = pre_angle;
            }
        }
        firstPredict = false;
        /*
        double px, vx, ax;
        double py, vy, ay;
        filter.kalman_Cal(&kalman_x, Predict_point.x, Speed[NowSpeed == 1 ? 0 : 1], acc, &px, &vx, &ax);
        filter.kalman_Cal(&kalman_y, Predict_point.y, Speed[NowSpeed == 1 ? 0 : 1], acc, &py, &vy, &ay);
        Predict_point.x = px;
        Predict_point.y = py;
        */
    }
    else {
        Predict_point.x = static_cast<int>(
            (x1 + (x2 - x1) * cos(-predict_rad * 3.14 / 180.0) - (y1 - y2) * sin(-predict_rad * 3.14 / 180.0)) / 100);//角度转弧度
        Predict_point.y = static_cast<int>(
            (y1 - (x2 - x1) * sin(-predict_rad * 3.14 / 180.0) - (y1 - y2) * cos(-predict_rad * 3.14 / 180.0)) / 100);
    }
}

void WindDetector::showPredictPoint(std::string windows_name,Mat src,Point PredictPoint) {
    cv::Point2f point = PredictPoint;
    cv::circle(src, point, 6, cv::Scalar(255, 0, 255));//在图像中画出特征点，2是圆的半径
    imshow(windows_name, src);
}

double WindDetector::CalPreRad() {
    if(SimpleMode){
        if(NowWindTime<-4.5) return 0.3392;
        else if(NowWindTime<-4) return 0.3386;
        else if(NowWindTime<-3.5) return 0.3391;
        else if(NowWindTime<-3) return 0.3395;
        else if(NowWindTime<-2.5) return 0.3389;
        else if(NowWindTime<-2) return 0.3394;
        else if(NowWindTime<-1.5) return 0.3389;
        else if(NowWindTime<-1) return 0.3393;
        else if(NowWindTime<-0.5) return 0.3393;
        else if(NowWindTime<0) return 0.3393;
        else if(NowWindTime<0.5) return 0.3393;
        else if(NowWindTime<1) return 0.3393;
        else if(NowWindTime<1.5) return 0.3394;
        else if(NowWindTime<2) return 0.3394;
        else if(NowWindTime<2.5) return 0.3394;
        else if(NowWindTime<3) return 0.3394;
        else if(NowWindTime<3.5) return 0.3395;
        else if(NowWindTime<4) return 0.3395;
        else if(NowWindTime<4.5) return 0.3395;
        else return 0.3391;
    }
    else{
        double t1, t2, rad1, rad2;
        t1 = NowWindTime; t2 = t1 + 0.544;
        rad1 = -0.785 / 1.884 * cos(1.884 * t1) + 1.305 * t1;
        rad2 = -0.785 / 1.884 * cos(1.884 * t2) + 1.305 * t2;
        //printf("rad:%.3lf\n", rad2-rad1);
        return rad2 - rad1;
    }
}

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void WindDetector::kalman1_filter_init(kalman1_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
        p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

float WindDetector::kalman1_filter_calc(kalman1_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;							  //输出预测结果x(k|k)
}

Filter1::Filter1()
{
    kalman_ok = 0;
}

Filter1::~Filter1()
{
}

///*********************************/

void Filter1::kalman2_init(kalman_t* kalman)
{
    if (kalman == &kalman_y)
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

        alloc_matrix_Init(&kalman->X, 3, 1);
        set_matrix(&kalman->X, 0.0, 0.0, 0.0);
        // print_matrix(kalman->X);
        alloc_matrix_Init(&kalman->X_mid, 3, 1);
        set_matrix(&kalman->X_mid, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->X_last, 3, 1);
        set_matrix(&kalman->X_last, 0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->K, 3, 3);
        set_matrix(&kalman->K, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->P, 3, 3);
        set_matrix(&kalman->P, 0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1);

        alloc_matrix_Init(&kalman->P_last, 3, 3);
        set_matrix(&kalman->P_last, 2.0, 0.0, 0.0,
            0.0, 2.0, 0.0,
            0.0, 0.0, 2.0);

        alloc_matrix_Init(&kalman->P_mid, 3, 3);
        set_matrix(&kalman->P_mid, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->Y, 3, 1);
        set_matrix(&kalman->Y, 0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->F, 3, 3);
        set_matrix(&kalman->F, 1.0, 1.0, 0.5,
            0.0, 1.0, 1.0,
            0.0, 0.0, 1.0);
        alloc_matrix_Init(&kalman->FT, 3, 3);
        set_matrix(&kalman->FT, 1.0, 0.0, 0.0,
            1.0, 1.0, 0.0,
            0.5, 1.0, 0.0);
        alloc_matrix_Init(&kalman->H, 3, 3);
        set_matrix(&kalman->H, 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);
        alloc_matrix_Init(&kalman->HT, 3, 3);
        set_matrix(&kalman->HT, 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);
        alloc_matrix_Init(&kalman->Q, 3, 3);
        set_matrix(&kalman->Q, 10.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 0.01);//系统噪声方差矩阵Q  一般取小值0.0001
                         //Q/R-->响应
        alloc_matrix_Init(&kalman->R, 3, 3);
        set_matrix(&kalman->R, 10.0, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 10.0);//测量噪声方差矩阵R

        alloc_matrix_Init(&kalman->T, 3, 3);
        set_matrix(&kalman->T, 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);

        alloc_matrix_Init(&kalman->buff1_33, 3, 3);
        set_matrix(&kalman->buff1_33, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff2_33, 3, 3);
        set_matrix(&kalman->buff2_33, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff3_33, 3, 3);
        set_matrix(&kalman->buff3_33, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff1_31, 3, 1);
        set_matrix(&kalman->buff1_31, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff2_31, 3, 1);
        set_matrix(&kalman->buff2_31, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff3_31, 3, 1);
        set_matrix(&kalman->buff3_31, 0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->buff1_13, 1, 3);
        set_matrix(&kalman->buff1_13, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff2_13, 1, 3);
        set_matrix(&kalman->buff2_13, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff3_13, 1, 3);
        set_matrix(&kalman->buff3_13, 0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->buff1_11, 1, 1);
        set_matrix(&kalman->buff1_11, 0.0);
        alloc_matrix_Init(&kalman->buff2_11, 1, 1);
        set_matrix(&kalman->buff2_11, 0.0);
        alloc_matrix_Init(&kalman->buff3_11, 1, 1);
        set_matrix(&kalman->buff3_11, 0.0);
    }
    else
    {
        alloc_matrix_Init(&kalman->X, 3, 1);
        set_matrix(&kalman->X, 0.0, 0.0, 0.0);
        // print_matrix(kalman->X);
        alloc_matrix_Init(&kalman->X_mid, 3, 1);
        set_matrix(&kalman->X_mid, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->X_last, 3, 1);
        set_matrix(&kalman->X_last, 0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->K, 3, 3);
        set_matrix(&kalman->K, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->P, 3, 3);
        set_matrix(&kalman->P, 0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1);

        alloc_matrix_Init(&kalman->P_last, 3, 3);
        set_matrix(&kalman->P_last, 2.0, 0.0, 0.0,
            0.0, 2.0, 0.0,
            0.0, 0.0, 2.0);

        alloc_matrix_Init(&kalman->P_mid, 3, 3);
        set_matrix(&kalman->P_mid, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->Y, 3, 1);
        set_matrix(&kalman->Y, 0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->F, 3, 3);
        set_matrix(&kalman->F, 1.0, 1.0, 0.5,
            0.0, 1.0, 1.0,
            0.0, 0.0, 1.0);
        alloc_matrix_Init(&kalman->FT, 3, 3);
        set_matrix(&kalman->FT, 1.0, 0.0, 0.0,
            1.0, 1.0, 0.0,
            0.5, 1.0, 0.0);
        alloc_matrix_Init(&kalman->H, 3, 3);
        set_matrix(&kalman->H, 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);
        alloc_matrix_Init(&kalman->HT, 3, 3);
        set_matrix(&kalman->HT, 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);
        alloc_matrix_Init(&kalman->Q, 3, 3);
        set_matrix(&kalman->Q, 10.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 0.05);//系统噪声方差矩阵Q  一般取小值0.0001

        alloc_matrix_Init(&kalman->R, 3, 3);
        set_matrix(&kalman->R, 10.0, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 1.0);//测量噪声方差矩阵R

        alloc_matrix_Init(&kalman->T, 3, 3);
        set_matrix(&kalman->T, 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);

        alloc_matrix_Init(&kalman->buff1_33, 3, 3);
        set_matrix(&kalman->buff1_33, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff2_33, 3, 3);
        set_matrix(&kalman->buff2_33, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff3_33, 3, 3);
        set_matrix(&kalman->buff3_33, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff1_31, 3, 1);
        set_matrix(&kalman->buff1_31, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff2_31, 3, 1);
        set_matrix(&kalman->buff2_31, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff3_31, 3, 1);
        set_matrix(&kalman->buff3_31, 0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->buff1_13, 1, 3);
        set_matrix(&kalman->buff1_13, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff2_13, 1, 3);
        set_matrix(&kalman->buff2_13, 0.0, 0.0, 0.0);
        alloc_matrix_Init(&kalman->buff3_13, 1, 3);
        set_matrix(&kalman->buff3_13, 0.0, 0.0, 0.0);

        alloc_matrix_Init(&kalman->buff1_11, 1, 1);
        set_matrix(&kalman->buff1_11, 0.0);
        alloc_matrix_Init(&kalman->buff2_11, 1, 1);
        set_matrix(&kalman->buff2_11, 0.0);
        alloc_matrix_Init(&kalman->buff3_11, 1, 1);
        set_matrix(&kalman->buff3_11, 0.0);
    }
    kalman_ok = 1;
}

float* Filter1::kalman_Cal(kalman_t* kalman, double p, double v, double a, double* filter_x, double* filter_v, double* filter_a)
{
    if (kalman_ok != 1)
        return NULL;

    //    kalman->X_last = kalman->X;
    //    kalman->P_last = kalman->P;
    //    memcpy(&kalman->X_last.data[0][0],&kalman->X.data[0][0],8);
    //    memcpy(&kalman->P_last.data[0][0],&kalman->P.data[0][0],16);
       // print_matrix(kalman->X);


    copy_matrix(kalman->X, kalman->X_last);
    copy_matrix(kalman->P, kalman->P_last);
    //    print_matrix(kalman->X);

        //test
        //destructive_invert_matrix(kalman->test1, kalman->test2);
    //    multiply_matrix((kalman->test1),(kalman->test2),kalman->end);
    //    print_matrix(kalman->end);

    multiply_matrix((kalman->F), (kalman->X_last), kalman->X_mid);
    // print_matrix(kalman->X_mid);

    multiply_matrix((kalman->F), (kalman->P_last), kalman->buff1_33);

    multiply_matrix((kalman->buff1_33), (kalman->FT), kalman->buff3_33);

    add_matrix((kalman->buff3_33), (kalman->Q), kalman->P_mid);

    multiply_matrix((kalman->H), (kalman->P_mid), kalman->buff3_33);
    multiply_matrix((kalman->buff3_33), (kalman->HT), kalman->buff2_33);

    add_matrix((kalman->buff2_33), (kalman->R), kalman->buff1_33);
    destructive_invert_matrix(kalman->buff1_33, kalman->buff3_33);

    multiply_matrix((kalman->P_mid), (kalman->HT), kalman->buff2_33);
    multiply_matrix((kalman->buff2_33), (kalman->buff3_33), kalman->K);

    set_matrix(&kalman->Y, p, v, a);

    //print_matrix(kalman->Y);
    multiply_matrix((kalman->H), (kalman->X_mid), kalman->buff1_31);
    subtract_matrix((kalman->Y), (kalman->buff1_31), kalman->buff2_31);
    multiply_matrix((kalman->K), (kalman->buff2_31), kalman->buff3_31);

    add_matrix((kalman->X_mid), (kalman->buff3_31), kalman->X);

    multiply_matrix((kalman->K), (kalman->H), kalman->buff1_33);
    multiply_matrix((kalman->T), (kalman->buff1_33), kalman->buff2_33);

    multiply_matrix((kalman->buff2_33), (kalman->P_mid), kalman->P);

    *filter_x = (float)kalman->X.data[0][0];
    *filter_v = (float)kalman->X.data[1][0];
    *filter_a = (float)kalman->X.data[2][0];

    printf(" filter_p : %f\n", *filter_x);
    printf(" filter_v : %f\n", *filter_v);
    printf(" filter_a : %f\n", *filter_a);

    printf("print->X\n");
    print_matrix(kalman->X);

    return NULL;
}
