#include"../General/General.h"
#include"../Armor/Armor.h"
#include"../AngleSolver/AngleSolver.h"
#include"../Serial/Serial.h"
#include"../Wind/Energy.h"
#include"../Can/Can.h"
#include"../Serial/predict.h"
#include "ros/ros.h"
#include"roborts_msgs/GimbalAngle.h"
#include"roborts_msgs/CameraAngle.h"
#include"roborts_msgs/ShootCmd.h"
#include"roborts_msgs/PyArmorInfo.h"
#include"roborts_msgs/GameStatus.h"
#include"roborts_msgs/FricWhl.h"
#include"roborts_msgs/RobotHeat.h"
#include"roborts_msgs/RobotAngle.h"
#include"actionlib/client/simple_action_client.h"
#include"roborts_msgs/GimbalCtrAction.h"
#include"roborts_msgs/GimbalSwingActionGoal.h"

//import detector
ArmorDetector Armor;
WindDetector Wind;

Filter filter_;

//import angle solver
AngleSolver angleSolver;

//Color ENEMYCOLOR = Color::RED;EnemyBlue 1
int  ENEMYCOLOR = 1;
//Mode mode = Mode::AUTO; 1
//Mode mode = Mode::BIG_WIND; 2
//Mode mode = Mode::SMALL_WIND; 3
int mode = 0;
float heat=0;
int bulletSpeed = 15000;
int roborts_id_=1;
int now_game_status=0;
float gimbal_yaw=0;
float gimbal_pitch=0;
float gimbal_angle=0;
int fric_speed=1300;

bool gimbalswing=false;


//Set armor detector prop
    ros::ServiceClient shoot_client;
    ros::ServiceClient fric_client;
 roborts_msgs::ShootCmd shoot_srv;
 void robot_angle_callback(const roborts_msgs::RobotAngle::ConstPtr & data)
{
    gimbal_angle=data->enemy_threa-data->self_threa;
    if(gimbal_angle>1.5*3.14 and gimbal_angle<2*3.14)
        gimbal_angle=gimbal_angle-2*3.14;
    else if(gimbal_angle<-1.5*3.14 and gimbal_angle>-2*3.14)
        gimbal_angle= 2*3.14+gimbal_angle;
}
 void camera_angle_callback(const roborts_msgs::CameraAngle::ConstPtr & camera)
{
    gimbal_yaw=(camera->yaw/3.14*180.0);
    gimbal_pitch=(camera->pitch/3.14*180.0);

}

 void heat_callback(const roborts_msgs::RobotHeat::ConstPtr & data)
{
    heat=data->shooter_heat;
    if(heat>=225)
    {
        //roborts_msgs::ShootCmd shoot_srv;
        shoot_srv.request.mode=0;
        shoot_srv.request.number=0;
        shoot_srv.request.freq=0;
        shoot_client.call(shoot_srv);
    }
}

 void game_callback(const roborts_msgs::GameStatus::ConstPtr & game)
{
    roborts_msgs::FricWhl fric_srv;
    if(game->game_status==4)
    {
        now_game_status=4;

        fric_srv.request.open=true;
        fric_srv.request.shootspeed=fric_speed;

        fric_client.call(fric_srv);
    }
    else if (game->game_status!=4)
    {
        now_game_status=game->game_status;

        fric_srv.request.open=false;
        fric_srv.request.shootspeed=fric_speed;

        fric_client.call(fric_srv);
    }
}

class ros_cv
{
   
public:
    ros::NodeHandle n;
    ros::Publisher cmd_gimbal_angle_pub = n.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
    ros::Publisher can_see_pub = n.advertise<roborts_msgs::PyArmorInfo>("PyArmorInfo", 1);
    ros::Subscriber robot_angle_sub;
    ros::Subscriber heat_sub;
    ros::Subscriber game_status_sub;
    ros::Subscriber camera_sub;
};

void armorDetectingThread()
{
    ros_cv roscv;
    shoot_client=roscv.n.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
    fric_client=roscv.n.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    roscv.robot_angle_sub=roscv.n.subscribe<roborts_msgs::RobotAngle>("robot_angle_pub",1,robot_angle_callback);
    roscv. heat_sub=roscv.n.subscribe<roborts_msgs::RobotHeat>("robot_heat",1,heat_callback);
    roscv.game_status_sub=roscv.n.subscribe<roborts_msgs::GameStatus>("game_status",1,game_callback);
    roscv.camera_sub=roscv.n.subscribe<roborts_msgs::CameraAngle>("camera_angle",1,camera_angle_callback);
    ros::param::get("robort_id",roborts_id_);

    if (roborts_id_==1 || roborts_id_==2)
    {
        ENEMYCOLOR=1;
    }
    else
    {
        ENEMYCOLOR=2;
    }
    Armor.loadSVM(SVM_PATH);
    Armor.setEnemyColor(ENEMYCOLOR); //here set enemy color

    //Set angle solver prop
    //hero:1 sentry:2 #3infantry:3 #4infantry2:4 139-02:5 139-06:6
    angleSolver.setCameraParam(CameraParam_PATH, 1);
    //hero:75 sentry:-40 #3infantry:80 #4infantry:40
    //angleSolver.setGunCamDistance(80);
    angleSolver.setArmorSize(ArmorType::SMALL_ARMOR, 135, 125);
    //angleSolver.setArmorSize(ArmorType::BIG_ARMOR,230,127);
    angleSolver.setBulletSpeed(bulletSpeed);
    this_thread::sleep_for(chrono::milliseconds(1000));

    double t,t1;
    int Number=0;
    int i = 0;
    int j=0;
    int _last_data = 0;
    //    static kalman_t* kalman_x;
    filter_.kalman2_init(&kalman_x, &kalman_yone, &kalman_filter_pitch);
    filter_.kalman2_init(&kalman_y, &kalman_yone, &kalman_filter_pitch);

    int far=0;
    int loss_enemy=0;
    actionlib::SimpleActionClient<roborts_msgs::GimbalCtrAction> gimbal_swing_action("gimbalswing",true);

    while(ros::ok()){
        // FPS
        t = cv::getTickCount();
        //consumer gets image

        if (1) {
            unique_lock<mutex> lck(Globalmutex);
            while (!imageReadable) {
                GlobalCondCV.wait(lck);
            }
            imageReadable = false;
        }
        //        if(receive_data.wind_change == 0){
        //            mode = 0;
        //        }else if(receive_data.wind_change == 1){
        //            mode = 1;
        //        }else if(receive_data.wind_change == 2){
        //            mdoe = 2;
        //        }

        vision_re _receive_data;



        mode = 0;

        if(mode == 0){
            Armor.setImg(src);
            //装甲板检测识别子核心集成函数
            Armor.run(src);
            //给角度解算传目标装甲板值的实例
            int X = 0,Y = 0,Z = 0;
            double yaw = 0,pitch = 0,distance = 0;
            if(Armor.isFoundArmor())
            {
                //  Point2f center_point;
                //                    Point2f _target_point = Armor.getSpinningPoint();
                //                    angleSolver.getAngle(_target_point, yaw, pitch);
                //                    pitch+=10;
                //                }
                //  else {
                ArmorBox target = Armor.getTarget();

                angleSolver.getAngle(target , X, Y, Z);

                // }
            
            if (gimbalswing)
            {
                gimbal_swing_action.cancelGoal();
                gimbalswing=false;
                continue;
            }

            vision_te test;
            distance=sqrt(X*X+Y*Y+Z*Z);
            //todo distance
	        _receive_data.little_vision.pitch_now=gimbal_pitch;
            _receive_data.little_vision.yaw_now=gimbal_yaw;
            

           
            if(distance>=5000)
            {
                far=6;
                _receive_data.little_vision.Bullet_speed=22;
                shoot_srv.request.number=1;
                shoot_srv.request.freq=1600;
                fric_speed=1330;
            }
            else if (distance<5000 && distance >=3500)
            {
                far=5;
                _receive_data.little_vision.Bullet_speed=21;
                shoot_srv.request.number=2;
                shoot_srv.request.freq=1600;
                fric_speed=1320;
            }
            else if (distance <3500 && distance >=3000)
            {
                far=4;
                _receive_data.little_vision.Bullet_speed=20;
                shoot_srv.request.number=3;
                shoot_srv.request.freq=1800;
                fric_speed=1305;
            }
            else if (distance <3000 && distance >=2000)
            {
                far=3;
                _receive_data.little_vision.Bullet_speed=18;
                shoot_srv.request.number=4;
                shoot_srv.request.freq=2000;
                fric_speed=1285;
            }
            else if (distance <2000 && distance >=1500)
            {
                far=2;
                _receive_data.little_vision.Bullet_speed=16;
                shoot_srv.request.number=5;
                shoot_srv.request.freq=2200;
                fric_speed=1265;
            }
            else 
            {
                far=1;
                _receive_data.little_vision.Bullet_speed=14;
                shoot_srv.request.number=8;
                shoot_srv.request.freq=2500;
                fric_speed=1250;
            }


            //_receive_data.little_vision.Bullet_speed

            vision_to_embedded(X, Y, Z, Armor.isFoundArmor(), Armor.isShoot(), &_receive_data.little_vision, &pitch ,&yaw , filter_, &test);
            //串口在此获取信息 yaw pitch distance，同时设定目标装甲板数字
            //            pitch = -(atan(Y / sqrt( X*X + Z*Z ))*180.0)/3.14159;


            roborts_msgs::GimbalAngle angle_msg;
            angle_msg.yaw_mode=false;
            angle_msg.pitch_mode=false;
            angle_msg.yaw_angle=yaw*3.14/180.0;
            angle_msg.pitch_angle=-pitch*3.14/180.0;
            roscv.cmd_gimbal_angle_pub.publish(angle_msg);

            shoot_srv.request.mode=1;
            shoot_client.call(shoot_srv);

            roborts_msgs::PyArmorInfo can_see_msg;
            can_see_msg.is_enemy=true;
            roscv.can_see_pub.publish(can_see_msg);

            


            


            // printf("if_find_armor : %f\t", Armor.isFoundArmor());
            // printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!autoshoot!!!!!!!!!!!!!!!!!!!!!!!!!!!\nyaw_now : %f\t", _receive_data.little_vision.yaw_now);
            // printf("pitch_now : %f\n", _receive_data.little_vision.pitch_now);
            // printf("pre_yaw : %f\t", test.pre_yaw);
            // printf("pre_pitch : %f\n", test.pre_pitch);
            loss_enemy=0;

            }
            else
            {
                roborts_msgs::PyArmorInfo can_see_msg;
                can_see_msg.is_enemy=false;
                roscv.can_see_pub.publish(can_see_msg);
                loss_enemy+=1;
                // if(gimbal_angle<3.14/2 and gimbal_angle>-3.14/2 and gimbal_angle!=0 and loss_enemy>50)
                // {
                //     if (!gimbalswing)
                //     {
                //         gimbal_swing_action.cancelGoal();
                //         gimbalswing=true;
                //     }
                //     roborts_msgs::GimbalAngle angle_msg;
                //     angle_msg.yaw_mode=false;
                //     angle_msg.pitch_mode=false;
                //     angle_msg.yaw_angle=gimbal_angle;
                //     angle_msg.pitch_angle=0.13;
                //     roscv.cmd_gimbal_angle_pub.publish(angle_msg);
                // }
                 if(loss_enemy>45)
                {
                    if (!gimbalswing)
                    {
                        roborts_msgs::GimbalCtrGoal goal;
                        goal.goal=true;
                        goal.angle=gimbal_yaw;
                        gimbal_swing_action.sendGoal(goal);

                        gimbalswing=true;
                    }
                    loss_enemy=0;
                }
            }


            if (1) {
                unique_lock <mutex> lck(SerialSendLock);
                send_data.yaw = yaw;
                send_data.pitch = pitch;
                send_data.distance = distance;
                send_data.filter = 0;
                send_data.find = Armor.isFoundArmor();
                sendNow = true;
                SerialSendCond.notify_one();
            }
//            serial.SerialSend(yaw, pitch, distance, test_filter, 0, Armor.isFoundArmor());

            //            Armor.send_message(&_receive_data);
            Armor.setEnemyColor(_receive_data.Enemy_color);

            //Armor.setROI();


            // if (mode != 1) break;
            //FPS
//             printf("if_find_armor : %f\t", Armor.isFoundArmor());
//             printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!autoshoot!!!!!!!!!!!!!!!!!!!!!!!!!!!\nyaw_now : %f\t", _receive_data.little_vision.yaw_now);
//             printf("pitch_now : %f\n", _receive_data.little_vision.pitch_now);
// //            printf("x_filter : %f\t", test.x_filter);
// //            printf("y_filter : %f\t", test.y_filter);
// //            printf("vx_filter : %f\t", test.vx_filter);
// //            printf("vy_filter : %f\n", test.vy_filter);
// //            printf("ax_filter : %f\t", test.ax_filter);
// //            printf("ay_filter : %f\t", test.ay_filter);
             //printf("pre_yaw : %f\t", yaw);
             //printf("pre_pitch : %f\n", pitch);
////            printf("X : %d\t\t\t", X);
//            printf("Y : %d\t\t\t", Y);+
//            printf("Z : %d\n", Z);
//            printf("armor_change : %d\t", Armor.isShoot());
//            printf("color : %d\n", _receive_data.Enemy_color);
//            printf(" shoot: %d\n", _receive_data.If_Press_Down);

            double t1 = (cv::getTickCount() - t) / cv::getTickFrequency();
            printf("!!!!!!!!!Image Acquiring FPS: %f\n", 1 / t1);
            /*if(Armor.isFoundArmor()){
                printf("Found Target! Center(%f,%f)\n", center_point.x, center_point.y);
                cout<<"Yaw: "<<yaw<<"Pitch: "<<pitch<<"Distance: "<<distance<<endl;
            }*/

#ifdef DEBUG_MODE
            //********************** DEGUG **********************//
            //装甲板检测识别调试参数是否输出
            //param:
            //		1.bool showSrcImg_ON,	  是否展示原图
            //      2.bool showSrcROI_ON ,     是否展示ROI
            //		3.bool showSrcBinary_ON,  是否展示二值图
            //		4.bool showLights_ON,	  是否展示灯条图
            //		5.bool showArmors_ON,	  是否展示装甲板图
            //		6.bool textLights_ON,	  是否输出灯条信息
            //		7.bool textArmors_ON,	  是否输出装甲板信息
            //		8.bool textScores_ON	  是否输出打击度信息
            // 				    1  2  3  4  5  6  7  8
            Armor.showDebugInfo(0, 0, 0, 0, 0, 0 , 0, 0);
            if(Armor.isFoundArmor())
            {
                //角度解算调试参数是否输出
                //param:
                //		1.showCurrentResult,	  是否展示当前解算结果
                //      2.bool ifWind             是否为大风车
                //		3.bool showTVec,          是否展示目标坐标
                //		4.bool showP4P,           是否展示P4P算法计算结果
                //		5.bool showPinHole,       是否展示PinHole算法计算结果
                //		6.bool showCompensation,  是否输出补偿结果
                //		7.bool showCameraParams	  是否输出相机参数
                //					      1  2  3  4  5  6  7
                angleSolver.showDebugInfo(0, 0, 0, 0, 0, 0, 0);
            }

            bool bRun = true;
            char chKey = waitKey(1);




#endif
        }
        else if (mode == 2) {
//            if(j<=40){
//                std::string imagename="/home/tars_go/Pictures/Wind1/"+std::to_string(j)+".bmp";
//                imwrite(imagename,src);
//            }

            j++;
            Wind.OpenBigWind = false;
            Wind.setEnemyColor(_receive_data.Enemy_color);
            Wind.setImg(src);
            //装甲板检测识别子核心集成函数
            Wind.run(src);
            //给角度解算传目标装甲板值的实例
            double yaw = 0, pitch = 0, distance = 0;
            Point2f center_point;
            if (Wind.isFoundArmor())
            {
                // vector<Point2f> contour_points;
                // ArmorType type = ArmorType::WIND_ARMOR;
                Wind.getTargetInfo(center_point);

                angleSolver.getAngle(center_point , yaw, pitch);
            }

            //串口在此获取信息 yaw pitch distance，同时设定目标装甲板数字
            SerialSendLock.lock();
            send_data.yaw = yaw;
            send_data.pitch = pitch;
            send_data.distance = distance;
            SerialSendLock.unlock();
            //FPS
            t1 = (cv::getTickCount() - t) / cv::getTickFrequency();
            printf("Small Wind Armor Detecting FPS: %f\n", 1 / t1);
            //            if (Wind.isFoundArmor()) {
            //                printf("Found Target! Center(%f,%f)\n", center_point.x, center_point.y);
            //                cout << "Yaw: " << yaw << "Pitch: " << pitch << "Distance: " << distance << endl;
            //            }

            //********************** DEGUG **********************//
            //装甲板检测识别调试参数是否输出
            //param:
            //		1.showSrcImg_ON,		          是否展示原图
            //		2.bool showSrcBinary_ON,          是否展示二值图
            //		3.bool showSrcBinaryArmFan_ON,	  是否展示扇叶二值图
            //		4.bool showArmors_ON,	          是否展示装甲板图
            //		5.bool showFlowStripFans_ON,	  是否展示流动条扇叶
            //		6.bool showTarget_ON,	          是否展示目标装甲板
            //		7.bool showcenterR_ON	          是否展示中心R
            //
            //			       1  2  3  4  5  6  7, 8
            Wind.showDebugInfo(0, 0, 0, 1, 0, 0, 0, 0);
            waitKey(1);

        }
        else if(mode == 1)
        {
            /*
            if(i<=40){
                std::string imagename="/home/tars_go/Pictures/Wind/"+std::to_string(i)+".bmp";
                imwrite(imagename,src);
            }
            i++;
            */

            Mat src1;
            for(;i<=40;i++){
                std::string imagename="/home/tars_go/Pictures/Wind2/"+std::to_string(i)+".bmp";
                src1 = imread(imagename);
            }
            //Mat src1 = imread("/home/tars_go/Pictures/Wind2/0.bmp");
            if(i==41)i = 0;
            Number++;//640,512,320-960,256-768,147-442
            Wind.OpenBigWind = true;
            Wind.setImg(src1);

            //装甲板检测识别子核心集成函数
            Wind.run(src1);
            //给角度解算传目标装甲板值的实例
            double yaw = 0, pitch = 0, distance = 0;
            Point2f center_point;
            if (Wind.isFoundArmor())
            {
                // vector<Point2f> contour_points;
                // ArmorType type = ArmorType::WIND_ARMOR;
                Wind.getTargetInfo(center_point);

                angleSolver.getAngle(center_point , yaw, pitch);
            }
            float h = -1.4 - 0.8 * sin(Wind.PredictAngle);
            float d = sqrt(7.5*7.5 + h*h +0.8 *0.8 - 2 * h * 0.8 *cos(3.14159 / 2 + Wind.PredictAngle));
            pitch = Vision_GravityCompensation(d,h, _receive_data.little_vision.Bullet_speed);//重力补偿

            //串口在此获取信息 yaw pitch distance，同时设定目标装甲板数字
            SerialSendLock.lock();
            send_data.yaw = yaw;
            send_data.pitch = pitch;
            send_data.distance = distance;
            SerialSendLock.unlock();
            //FPS
            t1 = (cv::getTickCount() - t) / cv::getTickFrequency();
            printf("Big Wind Armor Detecting FPS: %f\n", 1 / t1);
            //            if (Wind.isFoundArmor()) {
            //                printf("Found Target! Center(%f,%f)\n", center_point.x, center_point.y);
            //                cout << "Yaw: " << yaw << "Pitch: " << pitch << "Distance: " << distance << endl;
            //            }

            //********************** DEGUG **********************//
            //装甲板检测识别调试参数是否输出
            //param:
            //		1.showSrcImg_ON,		          是否展示原图
            //		2.bool showSrcBinary_ON,          是否展示二值图
            //		3.bool showSrcBinaryArmFan_ON,	  是否展示扇叶二值图
            //		4.bool showArmors_ON,	          是否展示装甲板图
            //		5.bool showFlowStripFans_ON,	  是否展示流动条扇叶
            //		6.bool showTarget_ON,	          是否展示目标装甲板
            //		7.bool showcenterR_ON	          是否展示中心R
            //
            //			       1  2  3  4  5  6  7, 8
            Wind.showDebugInfo(0, 0, 1, 1, 1, 0, 0, 0);
            waitKey(1);
 
        }
        ros::spinOnce();
    }
}
