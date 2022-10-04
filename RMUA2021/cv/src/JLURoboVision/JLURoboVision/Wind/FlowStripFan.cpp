#include"../Wind/Energy.h"

FlowStripFan::FlowStripFan(cv::RotatedRect fan_ROI) {
	center = fan_ROI.center;
	fanROI = fan_ROI;
	fan_ROI.points(fanVertices);
}

FlowStripFan::FlowStripFan()
{
	center = Point2f();
	fanROI = cv::RotatedRect();
}


FlowStripFan::~FlowStripFan() {}
