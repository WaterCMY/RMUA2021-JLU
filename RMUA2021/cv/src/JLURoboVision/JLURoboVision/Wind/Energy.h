/*
*	@Author: mmllllyyy
*	@Date:	 2021.3.20
*	@Brief:  This header file declares all the classes and params in the windmill project
*/

#ifndef ENERGY
#define ENERGY

#include "../General/General.h"
#include "../AngleSolver/filter.h"

using namespace cv;
class WindDetector;

enum class WindDetectorState
{
    ARMOR_NOT_FOUND = 0,
    ARMOR_FOUND = 1
};

//Matrix2f two_matrix_cross(Matrix2f end, Matrix2f one,Matrix2f two);

class Filter1
{
public:
    Filter1();
    ~Filter1();

    char kalman_ok;
    float* kalman_Cal(kalman_t* kalman, double p, double v, double a, double* filter_x, double* filter_v, double* filter_a);
    void kalman2_init(kalman_t* kalman);

};

typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
        float B;
    float Q;
    float R;
    float H;
}kalman1_t;

struct WindParam
{
    int color_threshold;   //color threshold for colorImg from substract channels 通道相减的colorImg使用的二值化阈值
    int bright_threshold;  //color threshold for brightImg 亮度图二值化阈值
    int RED_GRAY_THRESH;//敌方红色时的阈值
    int BLUE_GRAY_THRESH;//敌方蓝色时的阈值
    float Strip_Fan_Distance_min;//流动条到装甲板距离参数
    float Strip_Fan_Distance_max;

    bool show_energy;//是否显示图像
    bool show_process;//是否显示调试过程
    bool show_wrong;//是否显示报错
    bool show_data;//是否显示数据

    float armor_contour_area_max;//装甲板的相关筛选参数
    float armor_contour_area_min;
    float armor_contour_length_max;
    float armor_contour_length_min;
    float armor_contour_width_max;
    float armor_contour_width_min;
    float armor_contour_hw_ratio_max;
    float armor_contour_hw_ratio_min;

    float flow_strip_fan_contour_area_max;//流动条所在扇叶的相关筛选参数
    float flow_strip_fan_contour_area_min;
    float flow_strip_fan_contour_length_max;
    float flow_strip_fan_contour_length_min;
    float flow_strip_fan_contour_width_max;
    float flow_strip_fan_contour_width_min;
    float flow_strip_fan_contour_hw_ratio_max;
    float flow_strip_fan_contour_hw_ratio_min;
    float flow_strip_fan_contour_area_ratio_max;
    float flow_strip_fan_contour_area_ratio_min;

    long target_intersection_contour_area_min;//重合面积

    float Center_R_Control_area_max;//中心R的相关参数筛选
    float Center_R_Control_area_min;
    float Center_R_Control_radio_max;
    float Center_R_Control_radio_min;
    float Center_R_Control_area_radio_min;

    float flow_strip_contour_area_max;//流动条相关参数筛选
    float flow_strip_contour_area_min;
    float flow_strip_contour_length_max;
    float flow_strip_contour_length_min;
    float flow_strip_contour_width_max;
    float flow_strip_contour_width_min;
    float flow_strip_contour_hw_ratio_max;
    float flow_strip_contour_hw_ratio_min;
    float flow_strip_contour_area_ratio_min;
    float flow_strip_contour_intersection_area_min;

    WindParam() {
        color_threshold = 80;
        bright_threshold = 60;
        RED_GRAY_THRESH = 100;
        BLUE_GRAY_THRESH = 95;
        Strip_Fan_Distance_min = 28;
        Strip_Fan_Distance_max = 56;

        show_energy = true;//是否显示图像
        show_process = true;//是否显示调试过程
        show_wrong = false;//是否显示报错
        show_data = false;//是否显示数据

        armor_contour_area_max = 800; //装甲板的相关筛选参数
        armor_contour_area_min = 100;
        armor_contour_length_max = 50;
        armor_contour_length_min = 15;
        armor_contour_width_max = 30;
        armor_contour_width_min = 5;
        armor_contour_hw_ratio_max = 3;
        armor_contour_hw_ratio_min = 1.2;

        flow_strip_fan_contour_area_max = 6000;//流动条所在扇叶的相关筛选参数6000
        flow_strip_fan_contour_area_min = 2000;
        flow_strip_fan_contour_length_max = 150;//1 80
        flow_strip_fan_contour_length_min = 80;//80
        flow_strip_fan_contour_width_max = 90;//100
        flow_strip_fan_contour_width_min = 30;
        flow_strip_fan_contour_hw_ratio_max = 3;
        flow_strip_fan_contour_hw_ratio_min = 1.5;//1.5
        flow_strip_fan_contour_area_ratio_max = 0.8;
        flow_strip_fan_contour_area_ratio_min = 0.3;

        target_intersection_contour_area_min = 40;//重合面积

        Center_R_Control_area_max = 650;//中心R标筛选相关参数
        Center_R_Control_area_min = 150;
        Center_R_Control_radio_max = 1.3;
        Center_R_Control_radio_min = 0.7;
        Center_R_Control_area_radio_min = 0.5;

        flow_strip_contour_area_max = 2500;//流动条相关参数筛选
        flow_strip_contour_area_min = 400;
        flow_strip_contour_length_max = 100;
        flow_strip_contour_length_min = 30;//32
        flow_strip_contour_width_max = 40;//30
        flow_strip_contour_width_min = 10;
        flow_strip_contour_hw_ratio_min = 2.5;
        flow_strip_contour_hw_ratio_max = 10;
        flow_strip_contour_area_ratio_min = 0.4;
        flow_strip_contour_intersection_area_min = 10;
    }
};
extern WindParam windParam;


class WindArmorBox
{
public:
    friend WindDetector;
    WindArmorBox(cv::RotatedRect armor_ROI);
    WindArmorBox();
    ~WindArmorBox();

    void drawArmor(Mat& src);

private:
    Point2f armorVertices[4];  // bl->tl->tr->br     左下 左上 右上 右下
    Point2f center;	// center point(crossPoint) of armor 装甲板中心
    cv::RotatedRect armorROI;//armorROI区域的倾斜矩形
};


class FlowStripFan
{
public:
    friend WindDetector;
    FlowStripFan(cv::RotatedRect fan_ROI);
    FlowStripFan();
    ~FlowStripFan();
private:
    Point2f fanVertices[4];//bl->tl->tr->br     左下 左上 右上 右下
    Point2f center;	// center point(crossPoint) of fan 扇叶中心
    cv::RotatedRect fanROI;//armorROI区域的倾斜矩形
};



class WindDetector
{
public:
    WindDetector();
    ~WindDetector();

    /**
    * @brief: set enemyColor  设置敌方颜色
    * @param: enum class Color
    * @return: none
    */
    void setEnemyColor(int enemyColor);

    /**
    * @brief: set windMode  设置打击模式
    */
    void setDetectingMode(Mode windMode);

    /**
     *@brief: reset the WindDetector(delete the priviois data) to start next frame detection 重设检测器（删除原有数据），以便进行下一帧的检测
     */
    void resetDetector();

    /**
     *@brief: reset the WindDetector(delete the priviois data) to start next frame detection 重设检测器（删除原有数据），以便进行下一帧的检测
     */
    void setROI();

    /**
     * @brief: load source image and set roi if roiMode is open and found target in last frame 载入源图像并进行图像预处理
     * @param: const Mat& src     源图像的引用
     */
    void setImg(Mat& src);

    /**
     * @brief: find all the possible armors  检测所有可能的装甲板
     */
    void findArmors();

    /**
    * @brief: judge armor by size判断装甲板是否符合尺寸
    */
    bool isValidArmorContour(const vector<cv::Point>& armor_contour);

    /**
    * @brief: draw all the armors将当前容器所有装甲板画出
    */
    void drawArmor(std::string windows_name, Mat& src, vector<WindArmorBox> Armors);

    /**
     * @brief: find all the possible lights of armor 检测所有流动条扇叶
     */
    void findFlowStripFan();

    /**
    * @brief: judge flowstripfans by size判断流动条扇叶是否符合尺寸
    */
    bool isValidFlowStripFanContour(cv::Mat& src, const vector<cv::Point>& flow_strip_fan_contour);

    /**
    * @brief: draw all the fans将当前容器所有扇叶画出
    */
    void drawFan(std::string windows_name, Mat& src, vector<FlowStripFan> Fans);

    /**
     * @brief: find target 匹配正确目标
     */
    void matchTargetArmor();

    /**
     *@brief: an integrative function to run the Detector 集成的装甲板检测识别函数
     */
    void run(Mat& src);

    /**
     *@brief: return the Detector status 识别程序是否识别到装甲版
     *@return: FOUND(1) NOT_FOUND(0)
     */
    bool isFoundArmor();

    /**
     *@brief: get the vertices and type of target Armor for angle solver 将detector的结果输出
     */
    void getTargetInfo(Point2f& centerPoint);


    /**
     *@brief: show all the informations of this frame detection  显示所有信息
     */
    void showDebugInfo(bool showSrc_ON, bool showSrcBinary_ON, bool showSrcBinaryArmFan_ON, bool showArmors_ON, bool showFlowStripFans_ON, bool showTarget_ON, bool showcenterR_ON, bool showPredict_ON);


    /**
    * @brief: find the sign of R 检测R标
    */
    void findCenterR();

    /**
    * @brief: caculate the distance 计算两点间的距离
    */
    double pointDistance(cv::Point point_1, cv::Point point_2);

    /**
    * @brief: judge sign of R by size判断R标尺寸是否合格
    */
    bool isValidCenterRContour(const vector<cv::Point>& center_R_contour);

    /**
    * @brief: judge sign of R by size判断流动条尺寸是否合格
    */
    bool isValidFlowStripContour(const vector<cv::Point>& flow_strip_contour);

    /**
    /* @brief:This function obtains the polar coordinate angle of the target armor plate 此函数获取目标装甲板极坐标角度
    */
    void getTargetPolarAngle();

    /**
    * @brief: This function initializes the rotation direction of the energy mechanism此函数对能量机关旋转方向进行初始化
    */
    void initRotation();

    /**
    /* @brief:This function obtains the the target point 此函数获取预测点坐标
    */
    void getPredictPoint(cv::Point target_point);

    /**
    /* @brief:This function is used to calculate the predicted hitting point coordinates 此函数用于计算预测的击打点坐标
    */
    void rotate(cv::Point target_point);

    /**
    /* @brief:This function is used to display prediction points 此函数用于显示预测点
    */
    void showPredictPoint(std::string windows_name, Mat src, Point PredictPoint);

    /**
    /* @brief:This function is used to Calculate advance angle 此函数用于计算大幅预测提前角度
    */
    double CalPreRad();

    void kalman1_filter_init(kalman1_t *p,float T_Q,float T_R);

    float kalman1_filter_calc(kalman1_t* p,float dat);

    bool OpenBigWind;//是否开启大幅模式

    double PredictAngle;

private:
    int roiNext[4];
    Mat src_roi;
    Mat srcImg;  //source image (current frame acquired from camera) 从相机采集的当前的图像帧
    Mat srcImg_binary_armor; //binary image of srcImg 源图像的二值图(识别装甲板)
    Mat srcImg_binary_flow;//binary image of srcImg 源图像的二值图(识别流动条扇叶)
    Color enemyColor;  //the color of enemy 敌方颜色
    Mode windMode;  //wind detecting mode 识别模式
    WindDetectorState DetectorState;  //
    vector<FlowStripFan> candidateFans;  //
    vector<WindArmorBox> candidateArmors; //all of the armors 识别到的所有装甲板
    vector<WindArmorBox> targetArmors; //current target for current frame 当前图像帧对应的目标装甲板
    vector<FlowStripFan> targetFans; //current target for current frame 当前图像帧对应的目标扇叶
    std::vector<cv::RotatedRect> flow_strips;
    cv::Rect center_ROI; //the ROI of R R标的大致区域
    cv::RotatedRect centerR;//R标位置
    Point2f circle_center_point;//大风车中心点
    float predict_rad;//预测提前角
    float predict_rad_norm;//预测提前角的绝对值
    float target_polar_angle;//待击打装甲板的极坐标角度
    float last_target_polar_angle_judge_rotation;//   上一帧待击打装甲板的极坐标角度（用于判断旋向）
    bool energy_rotation_init;//若仍在判断风车旋转方向，则为true
    int energy_rotation_direction;//风车旋转方向
    int clockwise_rotation_init_cnt;//装甲板顺时针旋转次数
    int anticlockwise_rotation_init_cnt;//装甲板逆时针旋转次数
    Point Predict_point;//预测的击打点坐标
    double Time[3] = { 0 };//连续三帧的时间
    float Angle[3] = { 0 };//连续三帧的角度
    int NowTime;//当前时间所处位置
    bool FirstTime;//是否为识别速度的第一帧
    double Speed[2] = { 0 };//前后两次的速度
    int NowSpeed;//当前速度所处位置
    int NumSp;//计算速度的次数
    double NowWindTime;//当前大风车所处正弦函数的时间
    bool SimpleMode;
    kalman1_t * PreX, * PreY;
    kalman_t kalman_x, kalman_y;
    Filter1 filter;
    double acc;//角加速度
    bool firstPredict;
    double ChangeTime;
};

extern WindDetector windDetector;

#endif !ENERGY
