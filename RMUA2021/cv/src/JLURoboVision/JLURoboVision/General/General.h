/*
*	@Author: Qunshan He,mountain.he@qq.com
*	@Date:	 2021.03.16
*	@Brief:  This header file include the common head files and define the common structure, function and global variable.
*/

#ifndef GENERAL_H
#define GENERAL_H
#include<condition_variable>
#include<mutex>
#include<thread>
#include<vector>
#include<opencv4/opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
#include<cstring>
#include"../Serial/Serial.h"

using cv::Point;
using cv::Point2f;
using cv::Point3f;
using cv::Scalar;
using cv::Mat;
using cv::Size;
using cv::FONT_HERSHEY_SIMPLEX;
using cv::MORPH_RECT;
using cv::waitKey;
using std::vector;
using std::cout;
using std::endl;

//#define DEBUG_MODE
#define RELEASE_MODE

// extern variables
extern std::mutex Globalmutex;            // threads conflict due to image-updating
extern std::condition_variable GlobalCondCV;     // threads conflict due to image-updating
extern std::condition_variable SerialSendCond;
extern std::condition_variable SerialReceiveCond;
extern std::mutex SerialSendLock;
extern std::mutex SerialReceiveLock;
extern bool imageReadable;                  // threads conflict due to image-updating
extern bool sendNow;
extern bool receiveNow;
extern cv::Mat src;                         // Transfering buffer
extern Send_to_embedded send_data;
extern vision_re receive_data;

const char SVM_PATH[] = "/home/dji/JLURoboVision/JLURoboVision/General/svm1-9.xml";
const char CameraParam_PATH[] = "/home/dji/roborts_ws/src/RoboRTS/cv/src/JLURoboVision/JLURoboVision/General/camera_params.xml";
//const char SVM_PATH[] = "/home/tars-go/Desktop/JLURoboVision/JLURoboVision/General/svm1-9.xml";
//const char CameraParam_PATH[] = "/home/tars-go/Desktop/JLURoboVision/JLURoboVision/General/camera_params.xml";
//const char SVM_PATH[] = "F:/code/JLURoboVision/JLURoboVision/General/svm1-9.xml";
//const char CameraParam_PATH[] = "F:/code/JLURoboVision/JLURoboVision/General/camera_params.xml";

//set SN
const char SN_hero[] = "KE0200010112";           //139-03
const char SN_sentry_above[] = "KE0200010113";   //X
const char SN_sentry_below[] = "KE0200010109";   //O
const char SN_infantry_1[] = "KE0200020201";     //139-01
const char SN_infantry_2[] = "KE0200020202";     //139-02
const char SN_06[] = "KE0200010110";             //139-06



//int init();
//int childinit();
//void *ChildProcessFunc(void *);



/**
* @brief: imageUpdating thread using camera
*/
int imageUpdatingThreadCamera();

/**
* @brief: imageUpdating thread using local video
*/
int imageUpdatingThreadLocal();

/**
* @brief: armorDetecting thread
*/
void armorDetectingThread();

int SerialReceive();
int SerialSend();


/**
 *@brief: the types of armor BIG SMALL 大装甲板 小装甲板
 */
enum class ArmorType
{
    SMALL_ARMOR = 0,
    BIG_ARMOR = 1,
    WIND_ARMOR = 2
};

/**
* @brief: colors in order B G R 颜色B蓝 G绿 R红
*/
enum class Color
{
    BLUE = 0,
    GREEN = 1,
    RED = 2
};

/**
* @brief: shoot mode MANUAL0 ARMOR1 BIG_WIND2 SMALL_WIND3
*/
enum class Mode
{
    MANUAL = 0,
    AUTO = 1,
    BIG_WIND = 2,
    SMALL_WIND = 3
};

/*
 *@brief: get the distance of two points(a and b) 获取两点之间的距离 
 */
inline float getPointsDistance(const Point2f& a, const Point2f& b)
{
	float delta_x = a.x - b.x;
	float delta_y = a.y - b.y;
	//return sqrtf(delta_x * delta_x + delta_y * delta_y);
	return sqrt(delta_x * delta_x + delta_y * delta_y);
}


#endif // GENERAL_H
