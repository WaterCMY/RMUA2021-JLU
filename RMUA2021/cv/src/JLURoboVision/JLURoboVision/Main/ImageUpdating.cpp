#include "../GxCamera/GxCamera.h"
#include "../General/General.h"

#include "ros/ros.h"
/*GxCamera camera;*/             // import Galaxy Camera
extern cv::Mat src;          // Transfering buffer



int imageUpdatingThreadCamera()
{
	GX_STATUS status = GX_STATUS_SUCCESS;

	/*
	 *Preparation: CvMat image content
	*/
	cv::Mat frame;

	/*
	 *First init: Implementation of GxCamera and init it
	*/
	GxCamera gxCam;
	status = gxCam.initLib();
	GX_VERIFY(status);

	/*
	 *Second init: Open Camera by SN/Index
	*/
     //status = gxCam.openDeviceBySN(SN_infantry_2);	//By SN
   status = gxCam.openDeviceByIndex("1");		//By Index
	GX_VERIFY(status);

	/*
	 *Third init: Set Camera Params: ROI, Exposure, Gain, WhiteBalance
	*/
	gxCam.setRoiParam(800, 600, 0, 0);				// ROI
    gxCam.setExposureParam(500, false, 10000, 30000);	// Exposure2000
	gxCam.setGainParam(0, false, 0, 10);				// Gain
	gxCam.setWhiteBalanceOn(true);						// WhiteBalance
	/*
	 *Before acq: Send Acquisition Start Command
	*/
	status = gxCam.startAcquiring();					// Send Start Acquisition Command
	GX_VERIFY(status);

	while (ros::ok())
	{
		// FPS
		double t = cv::getTickCount();
		/*
		 *In acq: Snap a CvMat Image and store it in CvMat Content
		*/
		status = gxCam.snapCvMat(frame);
		GX_VERIFY(status);
		// Update the image acquired to src Mat content
		if (1) {
            unique_lock <mutex> lck(Globalmutex);
			frame.copyTo(src);
			imageReadable = true;
            GlobalCondCV.notify_one();
		}
//		char chKey = waitKey(1);
//		if (chKey == 'w' || chKey == 'W')
//			break;
		//FPS
//		double t1 = (cv::getTickCount() - t) / cv::getTickFrequency();
//        printf("Image Acquiring FPS: %f\n", 1 / t1);
	}

	/*
	 *After acq: Send Acquisition Stop Command
	*/
	status = gxCam.stopAcquiring();
	GX_VERIFY(status);

	/*
	*Close camera, while you can still open a new different camera
	*/
	gxCam.closeDevice();

	/*
	*Close lib: you can not use any GxCamera device unless you initLib() again
	*/
	gxCam.closeLib();
}

int imageUpdatingThreadLocal()
{ 
	char PATH[] = "F:/code/armor001.mp4";
	Mat frame;
	cv::VideoCapture capture;
	capture.open(PATH);

	/*if (!capture.isOpened())
	{
		printf("can not open ...\n");
		return -1;
	}*/

	while (capture.read(frame))
	{
		double t = cv::getTickCount();
		if (1) {
			unique_lock <mutex> lck(Globalmutex);
			frame.copyTo(src);
			//Mat show;
			//frame.copyTo(show);
			//imshow("frame", show);
			imageReadable = true;
			GlobalCondCV.notify_one();
		}
		double t1 = (cv::getTickCount() - t) / cv::getTickFrequency();
		printf("Image Acquiring FPS: %f\n", 1 / t1);
		waitKey(100);
	}
	capture.release();
	return 0;
}
