#include"../Wind/Energy.h"

void WindDetector::findArmors()
{
    static Mat dst;
    dst = srcImg_binary_armor.clone();
    std::vector<vector<Point> > armor_contours;
    std::vector<vector<Point> > armor_contours_external;//用总轮廓减去外轮廓，只保留内轮廓，除去流动条的影响。

    findContours(dst, armor_contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    findContours(dst, armor_contours_external, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (int i = 0; i < armor_contours_external.size(); i++)//去除外轮廓
    {
        int external_contour_size = armor_contours_external[i].size();
        for (int j = 0; j < armor_contours.size(); j++) {
            int all_size = armor_contours[j].size();
            if (external_contour_size == all_size) {
                swap(armor_contours[j], armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();//清除掉流动条
                break;
            }
        }

    }
    for (auto armor_contour : armor_contours) {
        if (!WindDetector::isValidArmorContour(armor_contour)) {
            continue;
        }
        /*
        double cur_contour_area = contourArea(armor_contour);
        cv::RotatedRect cur_rect = minAreaRect(armor_contour);
        cv::Size2f cur_size = cur_rect.size;
        float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
        float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
        float length_width_ratio = length / width;
        printf("area:%.2f length:%.2f width:%.2f hwratio:%.2f\n", cur_contour_area, length, width, length_width_ratio);
        */
        candidateArmors.emplace_back(cv::minAreaRect(armor_contour));//回传所有装甲板到armors容器中
    }
}

void WindDetector::findFlowStripFan()
{
    static Mat src_bin;
    src_bin = srcImg_binary_flow.clone();
    vector<vector<Point> > flow_strip_fan_contours;
    findContours(src_bin, flow_strip_fan_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    vector<cv::RotatedRect> candidate_flow_strip_fans;

    for (auto& flow_strip_fan_contour : flow_strip_fan_contours) {
        if (!isValidFlowStripFanContour(src_bin, flow_strip_fan_contour)) {
            continue;
        }

        /*
        double cur_contour_area1 = contourArea(flow_strip_fan_contour);
        cv::RotatedRect cur_rect1 = minAreaRect(flow_strip_fan_contour);
        cv::Size2f cur_size1 = cur_rect1.size;
        float length1 = cur_size1.height > cur_size1.width ? cur_size1.height : cur_size1.width;
        float width1 = cur_size1.height < cur_size1.width ? cur_size1.height : cur_size1.width;
        float length_width_ratio1 = length1 / width1;
        float area_ratio = cur_contour_area1 / cur_size1.area();
        printf("area:%.2f  length:%.2f  width:%.2f  lwratio:%.2f  arearatio:%.2f\n", cur_contour_area1, length1, width1, length_width_ratio1, area_ratio);
        */
        candidateFans.emplace_back(cv::minAreaRect(flow_strip_fan_contour));
    }
}

void WindDetector::matchTargetArmor()
{
    for (auto candidate_flow_strip_fan : candidateFans) {
        for (auto armor : candidateArmors) {
            std::vector<cv::Point2f> intersection;
            if (rotatedRectangleIntersection(armor.armorROI, candidate_flow_strip_fan.fanROI, intersection) == 0)
                continue;//此函数为opencv中寻找两个旋转矩形的交叉部分的API（即判断重合面积是否为0）
            double cur_contour_area = contourArea(intersection);
            if (cur_contour_area > windParam.target_intersection_contour_area_min) {
                targetArmors.emplace_back(armor);
                targetFans.emplace_back(candidate_flow_strip_fan);
                //cout << "Armor:" << target_armor.center << " ";
            }//返回目标装甲板参数
        }
    }//筛选出第一轮可能的目标扇叶和装甲板

    Mat src_bin;
    srcImg_binary_flow.copyTo(src_bin);
    for (auto candidate_target_armor : targetArmors) {
        cv::RotatedRect armorROI = candidate_target_armor.armorROI;
        Point2f vertices[4];
        armorROI.size.height *= 1.3;
        armorROI.size.width *= 1.3;
        armorROI.points(vertices);   //计算矩形的4个顶点
        for (int i = 0; i < 4; i++) {
            line(src_bin, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 0), 35);
        }
    }

    //if (windParam.show_process)imshow("flow strip struct", src_bin);

    std::vector<vector<Point> > flow_strip_contours;
    findContours(src_bin, flow_strip_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    for (auto& candidate_flow_strip_fan : targetFans) {
        for (auto& flow_strip_contour : flow_strip_contours) {
            if (!isValidFlowStripContour(flow_strip_contour)) {
                continue;
            }
            double min=100000;
            int id=100;
            cv::RotatedRect cur_rect = minAreaRect(flow_strip_contour);
            flow_strips.clear();
            flow_strips.emplace_back(cur_rect);
            for (int i = 0; i < targetArmors.size(); ++i) {
                double x=pointDistance(cur_rect.center,targetArmors[i].center);
                if(x<min) {
                    min=x;id=i;
                }
            }
            if(id<targetArmors.size()){
                WindArmorBox armor=targetArmors[id];
                targetArmors.clear();
                targetArmors.emplace_back(armor);
                break;
            }


            /*
            double cur_contour_area = contourArea(flow_strip_contour);
            cv::RotatedRect cur_rect1 = minAreaRect(flow_strip_contour);
            cv::Size2f cur_size = cur_rect1.size;
            float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
            float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
            float length_width_aim = length / width;
            float area_ratio = cur_contour_area / cur_size.area();
            printf("area:%.2f length:%.2f width:%.2f lwaid:%.2f arearatio:%.2f\n", cur_contour_area, length, width, length_width_aim, area_ratio);
            */

        }
    }//找到流动条并根据流动条删选出唯一正确的目标扇叶和装甲板

    if (targetArmors.empty()) {
        if (windParam.show_wrong) cout << "find target armor false" << endl;
    }
    else
        DetectorState = WindDetectorState::ARMOR_FOUND;
    return;
}

void WindDetector::findCenterR() {
    if (targetArmors.empty() || flow_strips.empty()) return;
    //大致框出R所在范围，提高识别精度，加快速度
    float length = targetArmors[0].armorROI.size.height > targetArmors[0].armorROI.size.width ?
        targetArmors[0].armorROI.size.height : targetArmors[0].armorROI.size.width;

    Point2f p2p(flow_strips[0].center.x - targetArmors[0].center.x, flow_strips[0].center.y - targetArmors[0].center.y);
    p2p = p2p / pointDistance(flow_strips[0].center, targetArmors[0].center);
    p2p = flow_strips[0].center + p2p * length * 1.7;
    float x = p2p.x - length, y = p2p.y - length;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    center_ROI = cv::Rect2f(x, y, length * 2, length * 2);

    //将R标的大致区域截取出来进行识别
    Mat sign_R = Mat::zeros(center_ROI.height, center_ROI.width, CV_8UC1);
    for (int i = center_ROI.y; i <= center_ROI.y + center_ROI.height - 1; i++)
        for (int j = center_ROI.x; j <= center_ROI.x + center_ROI.width - 1; j++)
            sign_R.at<uchar>(i - center_ROI.y, j - center_ROI.x) = srcImg_binary_flow.at<uchar>(i, j);
    std::vector<vector<Point> > center_R_contours;
    findContours(sign_R, center_R_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (auto& center_R_contour : center_R_contours) {
        if (!isValidCenterRContour(center_R_contour)) {
            continue;
        }
        centerR = cv::minAreaRect(center_R_contour);
        centerR = cv::RotatedRect(centerR.center + (Point2f)center_ROI.tl(), centerR.size, centerR.angle);
        float target_length =
            targetArmors[0].armorROI.size.height > targetArmors[0].armorROI.size.width ? targetArmors[0].armorROI.size.height : targetArmors[0].armorROI.size.width;
        circle_center_point = centerR.center;
        circle_center_point.y += target_length / 7.5;//实际最小二乘得到的中心在R的下方
        break;
    }
}
