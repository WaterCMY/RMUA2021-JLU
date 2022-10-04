#include"../Wind/Energy.h"

#include "../AngleSolver/filter.h"

WindParam windParam = WindParam();
WindDetector windDetector = WindDetector();

WindDetector::WindDetector()
{
    DetectorState = WindDetectorState::ARMOR_NOT_FOUND;
    last_target_polar_angle_judge_rotation = -1000;//上一帧待击打装甲板的极坐标角度（用于判断旋向）
    clockwise_rotation_init_cnt = 0;//装甲板顺时针旋转次数
    anticlockwise_rotation_init_cnt = 0;//装甲板逆时针旋转次数
    energy_rotation_init = true;//若仍在判断风车旋转方向，则为true
    predict_rad = 0;//预测提前角
    predict_rad_norm = 35;//预测提前角的绝对值角度
    Predict_point = Point(0, 0);//预测打击点初始化
    NowTime = 0;
    OpenBigWind = false;
    FirstTime = true;
    NowSpeed = 0;
    NumSp = 0;
    NowWindTime = 0;
    SimpleMode = false;
    PreX=(kalman1_t *)malloc(sizeof (kalman1_t));
    PreY=(kalman1_t *)malloc(sizeof (kalman1_t));
    kalman1_filter_init(PreX,10,10);
    kalman1_filter_init(PreY,10,10);
    filter.kalman2_init(&kalman_x);
    filter.kalman2_init(&kalman_y);
    firstPredict = true;
    ChangeTime = 0;
    PredictAngle = 0;
}

WindDetector::~WindDetector() {}

void WindDetector::resetDetector()
{
    DetectorState = WindDetectorState::ARMOR_NOT_FOUND;
    candidateArmors.clear();
    candidateFans.clear();
    targetArmors.clear();
    targetFans.clear();

}

void WindDetector::setROI()
{
    if (DetectorState == WindDetectorState::ARMOR_FOUND) {
        int col_begin = roiNext[0]; int row_begin = roiNext[2];
        roiNext[0] = col_begin + centerR.center.y - 240; roiNext[1] = col_begin + centerR.center.y + 240; roiNext[2] = row_begin + centerR.center.x - 320; roiNext[3] = row_begin + centerR.center.x + 320;//set roi of next frame
        //roiNext[0] =targetArmor.center.y - 240; roiNext[1] = targetArmor.center.y + 240; roiNext[2] = targetArmor.center.x - 320; roiNext[3] = targetArmor.center.x + 320;//set roi of next frame
        if (roiNext[0] < 320) roiNext[0] = 320; if (roiNext[1] > 1024) roiNext[1] = 1024;
        if (roiNext[2] < 0) roiNext[2] = 0; if (roiNext[3] > 1280) roiNext[3] = 1280;
    }
    else {
        roiNext[0] = 320; roiNext[1] = 1024; roiNext[2] = 0; roiNext[3] = 1280;//set roi of next frame
    }
}

void WindDetector::setEnemyColor(int enemyColor)
{
    if(enemyColor == 1)
    {
        this->enemyColor = Color::BLUE;
    }
    else if (enemyColor == 2)
    {
        this->enemyColor = Color::RED;
    }
}

void WindDetector::setDetectingMode(Mode windMode)
{
    this->windMode = windMode;
}
void WindDetector::getTargetInfo(Point2f& centerPoint)
{
    for (auto targetArmor : targetArmors) {
        centerPoint = targetArmor.center;
        return;
    }
}

void WindDetector::setImg(Mat& src)
{
    src.copyTo(srcImg);
    src.copyTo(srcImg_binary_armor);
    /*
    Mat hsvImage,dst1Image,dst2Image,HsvImage;
    cvtColor(src,hsvImage,COLOR_BGR2HSV);
    inRange(hsvImage,Scalar(156,43,46),Scalar(180,255,255),dst1Image);
    inRange(hsvImage,Scalar(0,43,46),Scalar(10,255,255),dst2Image);
    add(dst1Image,dst2Image,HsvImage);
    HsvImage.copyTo(srcImg_binary_armor);
    HsvImage.copyTo(srcImg_binary_flow);
    */
    /*
    Mat srcimg =Mat::zeros(srcImg.size(), CV_8UC1);
    uchar *pdata = (uchar*)srcImg.data;
    uchar *qdata = (uchar*)srcImg_binary_armor.data;
    uchar *qdata0 = (uchar*)srcimg.data;
    int srcData = srcImg.rows * srcImg.cols;
    int thresh=0;
    if (enemyColor == Color::BLUE)
    {
        for (int i = 0; i < srcData; i++)
        {
            *qdata0 =(*(pdata + 2) - *pdata);
            pdata += 3;
            qdata0++;
        }
        //获取自适应阈值
       thresh=threshold(srcimg,srcimg,0,255,cv::THRESH_OTSU);
       if (thresh<50)
       {
           return;
       }
       pdata=pdata-3*srcData;

       for (int i = 0; i < srcData; i++)
       {
           if (*(pdata + 2) - *pdata > thresh-20)
               *qdata = 255;
           pdata += 3;
           qdata++;
       }
    }
    else if (enemyColor == Color::RED)
    {

        for (int i = 0; i < srcData; i++)
        {
            *qdata0 =(*pdata - *(pdata+2));
            pdata += 3;
            qdata0++;
        }
       thresh=threshold(srcimg,srcimg,0,255,cv::THRESH_OTSU);
       if (thresh<50)
       {
           return;
       }
       pdata=pdata-3 * srcData;

        for (int i = 0; i < srcData; i++)
        {
            if (*pdata - *(pdata+2) > thresh-20)
                *qdata = 255;
            pdata += 3;
            qdata++;
        }
    }
        srcImg_binary_armor.copyTo(srcImg_binary_flow);

    */


    if (src.type() == CV_8UC3) {
        cvtColor(srcImg_binary_armor, srcImg_binary_armor, cv::COLOR_BGR2GRAY);
    }
    if (enemyColor == Color::BLUE) {
        threshold(srcImg_binary_armor, srcImg_binary_armor, 0, 255, cv::THRESH_OTSU);//windParam.RED_GRAY_THRESH cv::THRESH_BINARY
    }
    else if (enemyColor == Color::RED) {
        threshold(srcImg_binary_armor, srcImg_binary_armor, 0, 255, cv::THRESH_OTSU);
    }
    srcImg_binary_armor.copyTo(srcImg_binary_flow);

    /*
    Mat srcImg_binary;
    srcImg_binary = Mat::zeros(srcImg.size(), CV_8UC1); //color feature image
    int thresh;
    Mat srcimg =Mat::zeros(srcImg.size(), CV_8UC1);
    //pointer visits all the data of srcImg, the same to bgr channel split 通道相减法的自定义形式，利用指针访问，免去了split、substract和thresh操作，加速了1.7倍
    //data of Mat  bgr bgr bgr bgr
    uchar *pdata = (uchar*)srcImg.data;
    uchar *qdata = (uchar*)srcImg_binary.data;
    uchar *qdata0 = (uchar*)srcimg.data;
    int srcData = srcImg.rows * srcImg.cols;
    if (enemyColor == Color::RED)
    {
        for (int i = 0; i < srcData; i++)
        {
            *qdata0 =(*(pdata + 2) - *pdata);
            pdata += 3;
            qdata0++;
        }
        //获取自适应阈值
       thresh=35;
       pdata=pdata-3*srcData;

       for (int i = 0; i < srcData; i++)
       {
           if (*(pdata + 2) - *pdata > thresh-20)
               *qdata = 255;
           pdata += 3;
           qdata++;
       }
    }
    else if (enemyColor == Color::BLUE)
    {

        for (int i = 0; i < srcData; i++)
        {
            *qdata0 =(*pdata - *(pdata+2));
            pdata += 3;
            qdata0++;
        }
       thresh=35;
       pdata=pdata-3 * srcData;

        for (int i = 0; i < srcData; i++)
        {
            if (*pdata - *(pdata+2) > thresh-20)
                *qdata = 255;
            pdata += 3;
            qdata++;
        }
    }

    Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, Size(3, 3)); //kernel for dilate;  shape:ellipse size:Size(3,3) 膨胀操作使用的掩膜
    dilate(srcImg_binary, srcImg_binary, kernel); //dilate the roiImg_binary which can make the lightBar area more smooth 对roiIng_binary进行膨胀操作，试得灯条区域更加平滑有衔接
    cv::medianBlur(srcImg_binary, srcImg_binary, 3);
    srcImg_binary.copyTo(srcImg_binary_armor);
    srcImg_binary.copyTo(srcImg_binary_flow);
    */

    Mat element_dilate_1 = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat element_erode_1 = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(srcImg_binary_armor, srcImg_binary_armor, element_dilate_1);
    dilate(srcImg_binary_armor, srcImg_binary_armor, element_dilate_1);
    //dilate(srcImg_binary_armor, srcImg_binary_armor, element_dilate_1);
    //erode(srcImg_binary_armor, srcImg_binary_armor, element_erode_1);

    element_erode_1 = getStructuringElement(MORPH_RECT, Size(2, 2));
    Mat element_erode_2 = getStructuringElement(MORPH_RECT, Size(1, 1));
    element_dilate_1 = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(srcImg_binary_flow, srcImg_binary_flow, element_dilate_1);
    dilate(srcImg_binary_flow, srcImg_binary_flow, element_dilate_1);
    dilate(srcImg_binary_flow, srcImg_binary_flow, element_dilate_1);
    //erode(srcImg_binary_flow, srcImg_binary_flow, element_erode_1);
    //erode(srcImg_binary_flow, srcImg_binary_flow, element_erode_1);
    //erode(srcImg_binary_flow, srcImg_binary_flow, element_erode_2);
    //dilate(srcImg_binary_flow, srcImg_binary_flow, element_dilate_1);
}

bool WindDetector::isFoundArmor()
{
    if (this->DetectorState == WindDetectorState::ARMOR_FOUND)
        return true;
    else
        return false;
}

bool WindDetector::isValidArmorContour(const vector<cv::Point>& armor_contour)
{
    double cur_contour_area = contourArea(armor_contour);
    if (cur_contour_area > windParam.armor_contour_area_max ||
        cur_contour_area < windParam.armor_contour_area_min) {
        return false;
    }
    cv::RotatedRect cur_rect = minAreaRect(armor_contour);
    cv::Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if (length < windParam.armor_contour_length_min || width < windParam.armor_contour_width_min ||
        length >  windParam.armor_contour_length_max || width > windParam.armor_contour_width_max) {
        return false;
    }
    float length_width_ratio = length / width;
    if (length_width_ratio > windParam.armor_contour_hw_ratio_max ||
        length_width_ratio < windParam.armor_contour_hw_ratio_min) {
        return false;
    }
    return true;
}

bool WindDetector::isValidFlowStripFanContour(cv::Mat& src, const vector<cv::Point>& flow_strip_fan_contour) {
    double cur_contour_area = contourArea(flow_strip_fan_contour);
    if (cur_contour_area > windParam.flow_strip_fan_contour_area_max ||
        cur_contour_area < windParam.flow_strip_fan_contour_area_min) {
        return false;
    }
    cv::RotatedRect cur_rect = minAreaRect(flow_strip_fan_contour);
    cv::Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if (length < windParam.flow_strip_fan_contour_length_min
        || width < windParam.flow_strip_fan_contour_width_min
        || length > windParam.flow_strip_fan_contour_length_max
        || width > windParam.flow_strip_fan_contour_width_max) {
        return false;
    }
    float length_width_ratio = length / width;
    if (length_width_ratio > windParam.flow_strip_fan_contour_hw_ratio_max ||
        length_width_ratio < windParam.flow_strip_fan_contour_hw_ratio_min) {
        return false;
    }
    if (cur_contour_area / cur_size.area() < windParam.flow_strip_fan_contour_area_ratio_min
        || cur_contour_area / cur_size.area() > windParam.flow_strip_fan_contour_area_ratio_max) {
        return false;
    }
    return true;
}

void WindDetector::run(Mat& src)
{
    //first step, import image and pretreatment

    setImg(src);

    //second step, reset the detector
    resetDetector();

    //third step, find all of the possible armor, push to a vector "armors"
    findArmors();

    //fourth step,find all of the possible flowstripfans, push to a vector "candidatefans"
    findFlowStripFan();

    //fifth step,accorrding to the flowstrip,find the real flowstripfan and target armor
    matchTargetArmor();

    //sixth step,find the sign of R
    findCenterR();

    //seventh step,calculate the predict point
    if (isFoundArmor()) {
        getTargetPolarAngle();
        if (energy_rotation_init) {
            initRotation();
            return;
        }
        getPredictPoint(targetArmors[0].center);
    }
}

void WindDetector::showDebugInfo(bool showSrc_ON, bool showSrcBinary_ON, bool showSrcBinaryArmFan_ON, bool showArmors_ON, bool showFlowStripFans_ON, bool showTarget_ON, bool showcenterR_ON, bool showPredict_ON)
{
    if (showSrc_ON) imshow("binary", srcImg);

    if (showSrcBinary_ON) imshow("binary", srcImg_binary_armor);

    if (showSrcBinaryArmFan_ON) {
        imshow("armor_dilate", srcImg_binary_armor);
        imshow("flow_dilate", srcImg_binary_flow);
    }

    if (showArmors_ON) drawArmor("candidateArmors", srcImg, candidateArmors);

    if (showFlowStripFans_ON) drawFan("candidateFans", srcImg, candidateFans);

    if (showTarget_ON) drawArmor("targetArmors", srcImg, targetArmors);

    if (showcenterR_ON) {
        Mat dst, image2show;
        srcImg.copyTo(dst);
        srcImg.copyTo(image2show);
        rectangle(dst, center_ROI, cv::Scalar(0, 0, 255), 2);
        cv::imshow("centerROI", dst);
        Point2f vertices[4];      //定义矩形的4个顶点
        centerR.points(vertices);   ////计算矩形的4个顶点
        for (int i = 0; i < 4; i++)
            line(image2show, vertices[i], vertices[(i + 1) % 4], Scalar(255, 255, 0), 2);
        cv::circle(image2show, circle_center_point, 3, cv::Scalar(0, 0, 255), 2);
        cv::imshow("centerR", image2show);
    }

    if (showPredict_ON) showPredictPoint("predict", srcImg, Predict_point);
    /*
    Mat dst1;
    srcImg.copyTo(dst1);
    Point2f vertices1[4];
    flow_strips[0].points(vertices1);
    for (int i = 0; i < 4; i++)
        line(dst1, vertices1[i], vertices1[(i + 1) % 4], Scalar(255, 255, 0), 2);
    cv::imshow("flowstrip", dst1);
    */
}

void WindDetector::drawArmor(std::string windows_name, Mat& src, vector<WindArmorBox> Armors) {
    Mat dst;
    src.copyTo(dst);
    for (const auto& armor : Armors) {
        for (int i = 0; i < 4; i++)
            line(dst, armor.armorVertices[i], armor.armorVertices[(i + 1) % 4], Scalar(255, 0, 0), 2);
    }
    cv::imshow(windows_name, dst);
}

void WindDetector::drawFan(std::string windows_name, Mat& src, vector<FlowStripFan> Fans) {
    Mat image2show;
    src.copyTo(image2show);
    for (auto fan : Fans) {
        for (int i = 0; i < 4; i++)
            line(image2show, fan.fanVertices[i], fan.fanVertices[(i + 1) % 4], Scalar(127, 127, 255), 2);
    }
    /*
    double A = pointDistance(strip_fan_vertices[1], strip_fan_vertices[0]);
    double B = pointDistance(strip_fan_vertices[1], strip_fan_vertices[2]);
    std::cout << "lines" << A << "    wideth" << B << endl;
    */
    imshow(windows_name, image2show);
}

double WindDetector::pointDistance(cv::Point point_1, cv::Point point_2) {
    double distance = 0;
    distance = sqrt(pow(static_cast<double>(point_1.x - point_2.x), 2) + pow(static_cast<double>(point_1.y - point_2.y), 2));
    return distance;
}

bool WindDetector::isValidCenterRContour(const vector<cv::Point>& center_R_contour) {
    double cur_contour_area = contourArea(center_R_contour);
    if (cur_contour_area > windParam.Center_R_Control_area_max ||
        cur_contour_area < windParam.Center_R_Control_area_min) {
        //cout<<cur_contour_area<<" "<<energy_fan_param_.CONTOUR_AREA_MIN<<" "<<energy_fan_param_.CONTOUR_AREA_MAX<<endl;
        //cout<<"area fail."<<endl;
        return false;
        //选区面积大小不合适
    }
    cv::RotatedRect cur_rect = minAreaRect(center_R_contour);
    cv::Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if (cur_rect.center.x<center_ROI.width * 0.2 || cur_rect.center.x > center_ROI.width * 0.8 || cur_rect.center.y<center_ROI.height * 0.3 || cur_rect.center.y > center_ROI.height * 0.7) {
        return false;
    }
    float length_width_ratio = length / width;//计算矩形长宽比
    if (length_width_ratio > windParam.Center_R_Control_radio_max ||
        length_width_ratio < windParam.Center_R_Control_radio_min) {
        //cout<<"length width ratio fail."<<endl;
//        cout << "HW: " << length_width_ratio << '\t' << cur_rect.center << endl;
        return false;
        //长宽比不合适
    }
    if (cur_contour_area / cur_size.area() < windParam.Center_R_Control_area_radio_min) {
        //        cout << "area ratio: " << cur_contour_area / cur_size.area() << '\t' << cur_rect.center << endl;
        return false;//轮廓对矩形的面积占有率不合适
    }
    return true;
}

bool WindDetector::isValidFlowStripContour(const vector<cv::Point>& flow_strip_contour) {
    double cur_contour_area = contourArea(flow_strip_contour);
    if (cur_contour_area > windParam.flow_strip_contour_area_max || cur_contour_area < windParam.flow_strip_contour_area_min) {
        if (windParam.show_wrong) cout << "FlowStrip_cur_contour_area:" << cur_contour_area << endl;
        if (windParam.show_wrong) cout << "FlowStrip area fail." << endl;
        return false;
    }//流动条面积筛选

    cv::RotatedRect cur_rect = minAreaRect(flow_strip_contour);
    cv::Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    if (length > windParam.flow_strip_contour_length_max || width > windParam.flow_strip_contour_width_max
        || length < windParam.flow_strip_contour_length_min || width < windParam.flow_strip_contour_width_min) {
        if (windParam.show_wrong)cout << "length width fail." << endl;
        return false;
    }//流动条长宽筛选
    float length_width_aim = length / width;
    if (length_width_aim< windParam.flow_strip_contour_hw_ratio_min || length_width_aim> windParam.flow_strip_contour_hw_ratio_max) {
        if (windParam.show_wrong) cout << "length_width_aim fail." << endl;
        return false;
    }//长宽比筛选
    if (cur_contour_area / cur_size.area() < windParam.flow_strip_contour_area_ratio_min) {
        if (windParam.show_wrong) cout << "cur_contour_area / cur_size.area() fail." << endl;
        return false;
    }
    return true;
}
