/*
*	@Author: Mountain
*	@Date:	 2020.04.13
*	@Brief:  This cpp file define the ArmorNumClassifier class, realize some function used 
*/

#include "../Armor/Armor.h"
//设置透射变换后图片尺寸
ArmorNumClassifier::ArmorNumClassifier()
{
	svm = cv::ml::SVM::create();
	armorImgSize = Size(40, 40);
	p = Mat();
	
	warpPerspective_mat = Mat(3, 3, CV_32FC1);
	dstPoints[0] = Point2f(0, 0);
	dstPoints[1] = Point2f(armorImgSize.width, 0);
	dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
	dstPoints[3] = Point2f(0, armorImgSize.height);
}

ArmorNumClassifier::~ArmorNumClassifier(){}

void ArmorNumClassifier::loadSvmModel(const char * model_path, Size armorImgSize) {
	svm = cv::ml::StatModel::load<cv::ml::SVM>(model_path);
    if(svm.empty())
    {
        cout<<"Svm load error! Please check the path!"<<endl;
        exit(0);
    }
	this->armorImgSize = armorImgSize;

	//set dstPoints (the same to armorImgSize, as it can avoid resize armorImg)
	dstPoints[0] = Point2f(0, 0);
	dstPoints[1] = Point2f(armorImgSize.width, 0);
	dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
	dstPoints[3] = Point2f(0, armorImgSize.height);
}

void ArmorNumClassifier::loadImg(Mat & srcImg){

	//copy srcImg as warpPerspective_src
	(srcImg).copyTo(warpPerspective_src);

	//preprocess srcImg for the goal of acceleration

}

void ArmorNumClassifier::getArmorImg(ArmorBox & armor)
{	//Cut the original image according to the aspect ratio 按照固定长宽比采取图片，从而消除大小装甲板的影响
         srcPoints[0].x = ((armor.armorVertices[0].x)*5/6+(armor.armorVertices[1].x)/6);
         srcPoints[0].y = armor.armorVertices[0].y;
         srcPoints[1].x = ((armor.armorVertices[0].x)/6+(armor.armorVertices[1].x)*5/6);
         srcPoints[1].y = armor.armorVertices[1].y;
         srcPoints[2].x = ((armor.armorVertices[2].x)*5/6+(armor.armorVertices[3].x)/6);
         srcPoints[2].y = armor.armorVertices[2].y;
         srcPoints[3].x = ((armor.armorVertices[2].x)/6+(armor.armorVertices[3].x)*5/6);
         srcPoints[3].y = armor.armorVertices[3].y;


	//get the armor image using warpPerspective
	warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);  // get perspective transform matrix  透射变换矩阵
	warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, armorImgSize, cv::INTER_NEAREST, cv::BORDER_CONSTANT, Scalar(0)); //warpPerspective to get armorImage
    cvtColor( warpPerspective_dst,  warpPerspective_dst, 6);  //CV_BGR2GRAY=6
    threshold( warpPerspective_dst, warpPerspective_dst, 0, 255, cv::THRESH_OTSU);
    warpPerspective_dst.copyTo(armor.armorImg); //copyto armorImg
cv::imshow("roi",armor.armorImg);

    //std::string imagename="F:/9/"+std::to_string(i)+".bmp";
    //imwrite(imagename,warpPerspective_dst);
    //i++;
}

void ArmorNumClassifier::setArmorNum(ArmorBox & armor){

	// adapt armorImg to the SVM model sample-size requirement
	p = armor.armorImg.reshape(1, 1);
	p.convertTo(p, CV_32FC1);

	//set armor number according to the result of SVM  
	armor.armorNum = (int)svm->predict(p);


	armor.type = ArmorType::SMALL_ARMOR;
	

}
