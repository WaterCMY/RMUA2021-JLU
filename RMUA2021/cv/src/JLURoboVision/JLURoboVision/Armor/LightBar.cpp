/*
*	@Author: Mountain
*	@Date:	 2020.04.24
*	@Brief:  This cpp file define the object "light"
*/

#include "../Armor/Armor.h"


LightBar::LightBar(){
	lightRect = cv::RotatedRect();
	length = 0;
	center = Point2f();
	angle = 0;
}

LightBar::LightBar(const cv::RotatedRect &lightRect){
	this->lightRect = lightRect;
	length = MAX(lightRect.size.height, lightRect.size.width);
	center = lightRect.center;
	if (lightRect.angle > 90)
		angle = lightRect.angle - 180;
	else
		angle = lightRect.angle;
}

LightBar::~LightBar(){}
