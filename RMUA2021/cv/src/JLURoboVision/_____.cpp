#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
//按v采集视频，按p采集图片
//采集视频模式下，按c开始采集，按x停止采集
//采集图片模式下，按c开始采集，每次10张
int main() {
	bool flag = false;
	int pic_num = 10;    //每次采集图片数量
	VideoCapture cap(1);
	if (!cap.isOpened())
		return -1;
	Mat src, frame;
	for (;;) {
	    char d = getchar();
		if (d == 'v') {
			cap;
			int j = 1;
			cv::VideoWriter writer;
			cv::Size sWH = cv::Size((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));//用视频大小
			std::vector<String> filenames;
			
			string outputVideoAbsPath = "D:/JLUVision_Demos/video\\";   //路径需要更改
			
			string outputVideoname = to_string(j) + ".mp4";
			
			string outputVideoPath = outputVideoAbsPath + outputVideoname;
			
			cv::glob(outputVideoAbsPath, filenames);//获取路径下所有文件名
			
			
			for (int i = 0; i < filenames.size(); i++) {
				
				cout <<"???exist"<< filenames[i] << endl;
				cout << "outputVideoname:" << outputVideoPath << endl;
				
				if (outputVideoPath ==filenames[i])
					outputVideoPath = outputVideoAbsPath + to_string(++j) + ".mp4";
			}
			writer.open(outputVideoPath, VideoWriter::fourcc('M', 'P', '4', 'V'), 25.0, sWH);
			for (;;) {
				cap >> frame;
				if (flag) {
					if (frame.empty())break;
					frame.copyTo(src);
					writer << src;
					imshow("saving", src);
				}
				imshow("zzz", frame);
				char c = waitKey(10);
				//if (c == 's') system("pause");
				if (c == 'x') flag = false;
				if (c == 'c') flag = true;
				if (c == 27) {
					destroyWindow("zzz");
					break;
				}
			}
			writer.release();
			break;
		}

		else if (d == 'p') {
			cap;
			for (;;) {
				cap >> src;
				imshow("src", src);
				char c=waitKey(10);
				if (c == 'c') {
					for (int i = 0; i < pic_num; i++) {
						string img_path = "D:/JLUVision_Demos/picture" + to_string(i) + ".bmp";   //路径需要更改
						imwrite(img_path, src);
					}
				}
				if(c==27){
					destroyWindow("zzz");
					break;
				}
			}
			break;
		}
		else continue;
		//system("pause");
	}
}
//avi是MP42，mp4是mp4v（大小写均可）,size自己的定义的不能播放
