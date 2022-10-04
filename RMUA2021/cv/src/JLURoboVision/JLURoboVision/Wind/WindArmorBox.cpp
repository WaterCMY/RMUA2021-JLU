#include"../Wind/Energy.h"

WindArmorBox::WindArmorBox(cv::RotatedRect armor_ROI)
{
	center = armor_ROI.center;
	armorROI = armor_ROI;
	armor_ROI.points(armorVertices);
}

WindArmorBox::WindArmorBox()
{
	center = Point2f();
	armorROI = cv::RotatedRect();
}


WindArmorBox::~WindArmorBox(){}

void WindArmorBox::drawArmor(Mat& src)
{

}
