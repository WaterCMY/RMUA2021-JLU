#include "../Armor/Armor.h"

ArmorParam armorParam = ArmorParam();


ArmorDetector::ArmorDetector()
{
    state = DetectorState::LIGHTS_NOT_FOUND;
    roiNext[0] = 0; roiNext[1] = 600; roiNext[2] = 0; roiNext[3] = 800;
}

ArmorDetector::~ArmorDetector(){}

void ArmorDetector::resetDetector()
{
    state = DetectorState::LIGHTS_NOT_FOUND;
    lights.clear();
    armors.clear();
}

/**
* @brief: set enemyColor  设置敌方颜色
*/
void ArmorDetector::setEnemyColor(int  enemyColor)
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

void ArmorDetector::setPlayerNum(const int & targetNum)
{
    this->playerNum = targetNum;
    if(!targetNum){
        targetArmor = ArmorBox();
        state = DetectorState::ARMOR_NOT_FOUND;
    }
}


/**
* @brief: set ROI  设置ROI
*/
void ArmorDetector::setROI()
{/*
    if (state == DetectorState::ARMOR_FOUND) {
        int col_begin = roiNext[0]; int row_begin = roiNext[2];
        //roiNext[0] = targetArmor.center.y - 384; roiNext[1] = targetArmor.center.y + 384; roiNext[2] = targetArmor.center.x - 480; roiNext[3] = targetArmor.center.x + 480;//set roi of next frame
        //cout<<roiNext[2]<<" "<<roiNext[3]<<" "<<roiNext[0]<<" "<<roiNext[1]<<" "<<endl<<endl;
        roiNext[0] =targetArmor.center.y - 240; roiNext[1] = targetArmor.center.y + 240; roiNext[2] = targetArmor.center.x - 320; roiNext[3] = targetArmor.center.x + 320;//set roi of next frame
        //cout<<roiNext[0]<<" "<<roiNext[1]<<" "<<roiNext[2]<<" "<<roiNext[3]<<" "<<endl;
        if (roiNext[0] < 0) roiNext[0] = 0; if (roiNext[1] > 1024) roiNext[1] = 1024;
        if (roiNext[2] < 0) roiNext[2] = 0; if (roiNext[3] > 1280) roiNext[3] = 1280;
    }
    else {
        //cout<<"reset"<<endl;
        roiNext[0] = 0; roiNext[1] = 1024; roiNext[2] = 0; roiNext[3] = 1280;//set roi of next frame
    }*/
    roiNext[0] = 256; roiNext[1] = 768; roiNext[2] = 0; roiNext[3] = 1280;
    //cout<<roiNext[2]<<" "<<roiNext[3]<<" "<<roiNext[0]<<" "<<roiNext[1]<<" "<<endl<<endl;
}
/**
* @brief: load source image and set roi if roiMode is open and found target in last frame 载入源图像并设置ROI区域（当ROI模式开启，且上一帧找到目标装甲板时）
* @param: const Mat& src     源图像的引用
*/
void ArmorDetector::setImg(Mat & src){
    //src_roi = src(cv::Range(roiNext[0], roiNext[1]), cv::Range(roiNext[2], roiNext[3]));
    //src_roi = src;
    src.copyTo(src_full);  //deep copy src to srcImg 深（值）拷贝给srcImg
    classifier.loadImg(src_full); //srcImg for classifier, warp perspective  载入classifier类成员的srcImg，用于透射变换剪切出装甲板图
    src_full(cv::Range(roiNext[0], roiNext[1]), cv::Range(roiNext[2], roiNext[3])).copyTo(src_roi);
    src_roi.copyTo(srcImg);  //deep copy src to srcImg 深（值）拷贝给srcImg
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
    else if (enemyColor == Color::BLUE)
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

    Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, Size(3, 3)); //kernel for dilate;  shape:ellipse size:Size(3,3) 膨胀操作使用的掩膜
    dilate(srcImg_binary, srcImg_binary, kernel); //dilate the roiImg_binary which can make the lightBar area more smooth 对roiIng_binary进行膨胀操作，试得灯条区域更加平滑有衔接
    cv::medianBlur(srcImg_binary, srcImg_binary, 3);
}

/**
 *@brief: load SVM model 载入svm模型
 *@param: the path of svm xml file and the size of training images svm模型路径及训练集的图像大学
 */
void ArmorDetector::loadSVM(const char * model_path, Size armorImgSize)
{
    classifier.loadSvmModel(model_path, armorImgSize);
}


/**
 *@brief: an integrative function to run the Detector 集成跑ArmorDetector
 */
void ArmorDetector::run(Mat & src) {
    //firstly, load and set srcImg  首先，载入并处理图像
    setImg(src); //globally srcImg and preprocess it into srcImg_binary 载入Detector的全局源图像 并对源图像预处理成

    //secondly, reset detector before we findLights or matchArmors(clear lights and armors we found in the last frame and reset the state as LIGHTS_NOT_FOUND)
    //随后，重设detector的内容，清空在上一帧中找到的灯条和装甲板，同时检测器状态重置为LIGHTS_NOT_FOUND（最低状态）
    resetDetector();

    //thirdly, find all the lights in the current frame (srcImg)
    //第三步，在当前图像中找出所有的灯条
    findLights();

    //forthly, if the state is LIGHTS_FOUND (detector found more than two lights) , we match each two lights into an armor
    //第四步，如果状态为LIGHTS_FOUND（找到多于两个灯条），则
    if (state == DetectorState::LIGHTS_FOUND)
    {
        //match each two lights into an armor and if the armor is a suitable one, emplace back it into armors
        //将每两个灯条匹配为一个装甲板，如果匹配出来的装甲板是合适的，则压入armors中
        matchArmors();
        //如果找到了灯条，则设置好目标装甲板和上一个装甲板
        setTargetArmor();
        //if the state is ARMOR_FOUND(detector has matched suitable armor), set target armor and last armor
    }

    updateShootState();
}

ArmorBox ArmorDetector::getTarget()
{
    return targetArmor;
}

/**
 *@brief: return the Detector status 识别程序是否识别到装甲版
 *@return: FOUND(1) NOT_FOUND(0)
 */
bool ArmorDetector::isFoundArmor()
{
    if(state == DetectorState::ARMOR_FOUND)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}



int ArmorDetector::isShoot()
{
    switch (shoot){
    case ShootState::LOST:
        return 0; break;
    case ShootState::WAITING:
        return 1; break;
    case ShootState::FOLLOWING:
        return 2; break;
    case ShootState::SPINNING:
        return 3; break;
    case ShootState::ARMOR_SWITCH:
        return 4; break;
    case ShootState::CAR_SWITCH:
        return 5; break;
    }
}

///////////////////////////////////////////////////////////  functions  for   debugging      //////////////////////////////////////////////////////////////////

/**
 *@brief: show all the lights found in a copy of srcImg  在图像中显示找到的所有灯条
 */
void showLights(Mat & image, const vector<LightBar> & lights)
{
    Mat lightDisplay = Mat::zeros(image.size(), CV_8UC3);;//image for the use of dialaying the lights 显示灯条用的图像
    image.copyTo(lightDisplay);//get a copy of srcImg 获取源图像的拷贝
    //if detector finds lights 如果找到了灯条
    if (!lights.empty())
    {
        putText(lightDisplay, "LIGHTS FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 1, 8, false); //title LIGHT_FOUND 大标题 “找到了灯条”
        for (auto light : lights)
        {
            Point2f lightVertices[4];
            light.lightRect.points(lightVertices);
            //draw all the lights' contours 画出所有灯条的轮廓
            for (size_t i = 0; i < 4; i++)
            {
                line(lightDisplay, lightVertices[i], lightVertices[(i + 1) % 4], Scalar(255, 0, 255), 1, 8, 0);
            }

            //draw the lights's center point 画出灯条中心
            circle(lightDisplay, light.center, 2, Scalar(0, 255, 0), 2, 8, 0);

            //show the lights' center point x,y value 显示灯条的中心坐标点\角度\长度
            putText(lightDisplay, std::to_string(int(light.angle)), light.center - Point2f(0, 15), cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            putText(lightDisplay, std::to_string(int(light.center.x)), light.center, cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            putText(lightDisplay, std::to_string(int(light.center.y)), light.center + Point2f(0, 15), cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
            putText(lightDisplay, std::to_string(int(light.length)), light.center + Point2f(0, 30), cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
        }
    }
    //if detector does not find lights 如果没找到灯条
    else
    {
        putText(lightDisplay, "LIGHTS NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8, false);//title LIGHT_NOT_FOUND 大标题 “没找到灯条”
    }
    //show the result image 显示结果图
    imshow("Lights Monitor", lightDisplay);
}

/**
 *@brief: show all the armors matched in a copy of srcImg  在图像中显示找到的所有装甲板
 */
void showArmors(Mat & image, const vector<ArmorBox> & armors, const vector<LightBar>& lights,const ArmorBox & targetArmor)
{
//    if(armors.size()>1){
//        cout<<"????????????????????????????????????????????"<<endl;
//    }
    Mat armorDisplay = Mat::zeros(image.size(), CV_8UC3); //Image for the use of displaying armors 展示装甲板的图像
    image.copyTo(armorDisplay); //get a copy of srcImg 源图像的拷贝
    // if armors is not a empty vector (ARMOR_FOUND) 如果找到了装甲板
    if (!armors.empty())
    {
        putText(armorDisplay, "ARMOR FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 1, 8, false); //title FOUND 大标题 “找到了装甲板”
        //draw all the armors' vertices and center 画出所有装甲板的顶点边和中心
        for (auto armor : armors)
        {
            //draw the center 画中心
            circle(armorDisplay, armor.center, 2, Scalar(0, 255, 0), 2);
            for (size_t i = 0; i < 4; i++)
            {
                line(armorDisplay, armor.armorVertices[i], armor.armorVertices[(i + 1) % 4], Scalar(255, 255, 0), 2, 8, 0);
            }
            //display its center point x,y value 显示中点坐标
            putText(armorDisplay, std::to_string(int(armor.center.x)), armor.center, cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(armorDisplay, std::to_string(int(armor.center.y)), armor.center + Point2f(0, 15), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(armorDisplay, std::to_string(int(armor.armorNum)), armor.center + Point2f(15, 30), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, 8, false);
            double angle = std::atan2(std::fabs(lights[armor.l_index].center.y - lights[armor.r_index].center.y), lights[armor.r_index].center.x - lights[armor.l_index].center.x);
            double light_angle = (std::fabs(lights[armor.l_index].angle) + std::fabs(lights[armor.r_index].angle)) / 2 *CV_PI/180;
            putText(armorDisplay, std::to_string(angle), armor.center + Point2f(0, 45), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(armorDisplay, std::to_string(light_angle), armor.center + Point2f(0, 65), cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
        }
        //connect all the vertices to be the armor contour 画出装甲板轮廓
        for (size_t i = 0; i < 4; i++)
        {
            line(armorDisplay, targetArmor.armorVertices[i], targetArmor.armorVertices[(i + 1) % 4], Scalar(255, 255, 255), 2, 8, 0);
        }
    }
    //if armors is a empty vector (ARMOR_NOT FOUND) 如果没找到装甲板
    else
    {
        //cout << "armornotfound" << endl;
        putText(armorDisplay, "ARMOR NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);//title NOT FOUND 大标题 “没找到装甲板”
    }
    //show the result armors image 显示结果图
    imshow("Armor Monitor", armorDisplay);
}



/**
 *@brief: show all the lights information in console  在控制台输出找到灯条的中心和角度
 */
void textLights(vector<LightBar> & lights)
{
    cout << "\n################## L I G H T S ##################" << endl;
    if (lights.empty()) {
        cout << "LIGHTS NOT FOUND!" << endl;
    }
    else
    {
        cout << "LIGHTS FOUND!" << endl;
        for (size_t i = 0; i < lights.size(); i++)
        {
            cout << "#############################" << endl;
            cout << "Light Center:" << lights[i].center << endl;
            cout << "Light Angle:" << lights[i].angle << endl;
        }
        cout << "#################################################" << endl;
    }
}

/**
 *@brief: show all the armors information in console  在控制台输出找到装甲板的中心、数字、匹配信息
 */
void textArmors(vector<ArmorBox> & armors)
{
    cout << "\n$$$$$$$$$$$$$$$$$$ A R M O R S $$$$$$$$$$$$$$$$$$" << endl;
    if (armors.empty()) {
        cout << "ARMORS NOT FOUND!" << endl;
    }
    else
    {
        cout << "ARMOR FOUND!" << endl;
        for (size_t i = 0; i < armors.size(); i++)
        {
            cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
            cout << "Armor Center: " << armors[i].center << endl;
            cout << "Armor Number: " << armors[i].armorNum << endl;
            if (armors[i].type == ArmorType::SMALL_ARMOR) cout << "Armor Type: SMALL ARMOR" << endl;
            else if(armors[i].type == ArmorType::BIG_ARMOR) cout << "Armor Type: BIG ARMOR" << endl;
            cout << "\n###### matching information ######" << endl;
            cout << "Angle difference: " << armors[i].getAngleDiff() << endl;
            cout << "Deviation Angle: " << armors[i].getDeviationAngle() << endl;
            cout << "X Dislocation Ration: " << armors[i].getDislocationX() << endl;
            cout << "Y Dislocation Ration: " << armors[i].getDislocationY() << endl;
            cout << "Length Ration: " << armors[i].getLengthRation() << endl;
        }
        cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
    }

}

/**
 *@brief: show all the armors score information in console  在控制台输出找到装甲板的打击度信息
 */
void textScores(vector<ArmorBox> & armors, ArmorBox & lastArmor)
{
    if (!armors.empty())
    {
        cout << "\n@@@@@@@@@@@@@@@@@@ S C O R E S @@@@@@@@@@@@@@@@@@" << endl;
        for (size_t i = 0; i < armors.size(); i++)
        {
            float score = 0;  // shooting value of armor的打击度
            cout << "Armor Center: " << armors[i].center << endl;
            cout << "Area: " << armors[i].armorRect.area() << endl;
            score += armors[i].armorRect.area(); //area value of a a_armor面积得分


            if (lastArmor.armorNum != 0) {  //if lastArmor.armorRect is not a default armor means there is a true targetArmor in the last frame 上一帧图像中存在目标装甲板
                float a_distance = getPointsDistance(armors[i].center, lastArmor.center); //distance score to the lastArmor(if exist) 装甲板距离得分，算负分
                cout << "Distance: " << a_distance << endl;
                score -= a_distance * 2;
            }
        }
        cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
    }
}


void showpredictArmor(Point2f centerPre, vector<Point2f> armorPre, Mat img) {
    Mat image = Mat::zeros(img.size(), CV_8UC3);
    img.copyTo(image);
    //printf("armorPre size==============%d\n", armorPre.size());
    if(armorPre.size()>0)
        for (size_t i = 0; i < armorPre.size(); i++)
        {
            line(image, armorPre[i], armorPre[(i + 1) % 4], Scalar(255, 0, 0), 2, 8, 0);
        }
    circle(image, centerPre, 1, Scalar(0, 0, 255), 2, 8, 0);
    putText(image, std::to_string(int(centerPre.x)), centerPre, cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
    putText(image, std::to_string(int(centerPre.y)), centerPre + Point2f(0, 15), cv::FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
    imshow("predict", image);
}

/**
 *@brief: lights, armors, lights to armors every information in one 所有调试用数据输出
 */
void ArmorDetector::showDebugInfo(bool showSrcImg_ON, bool showSrcROI_ON, bool showSrcBinary_ON, bool showLights_ON, bool showArmors_ON, bool textLights_ON, bool textArmors_ON, bool textScores_ON)
{
    if (showSrcImg_ON)
        imshow("src", src);
    if (showSrcROI_ON)
        imshow("roi", srcImg);
    if (showSrcBinary_ON)
        imshow("srcImg_Binary", srcImg_binary);
    if (showLights_ON)
        showLights(src_full, lights);
    if (showArmors_ON)
        showArmors(src_full, armors, lights, targetArmor);
    if (textLights_ON)
        textLights(lights);
    if (textArmors_ON)
        textArmors(armors);
    if (textScores_ON)
        textScores(armors, lastArmor);
    //  //for predict
    // armorPredict.armorPredict_ON=armorPredict_ON_parm;

    //if (Kalmanpredict_ON)
    //    showpredictArmor(centerPre, armorPre, srcImg);


    /*if(isFoundArmor())
        imshow("Armor Binary",targetArmor.armorImg);*/
}
