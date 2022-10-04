#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
//��v�ɼ���Ƶ����p�ɼ�ͼƬ
//�ɼ���Ƶģʽ�£���c��ʼ�ɼ�����xֹͣ�ɼ�
//�ɼ�ͼƬģʽ�£���c��ʼ�ɼ���ÿ��10��
int main() {
	bool flag = false;
	int pic_num = 10;    //ÿ�βɼ�ͼƬ����
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
			cv::Size sWH = cv::Size((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));//����Ƶ��С
			std::vector<String> filenames;
			
			string outputVideoAbsPath = "D:/JLUVision_Demos/video\\";   //·����Ҫ����
			
			string outputVideoname = to_string(j) + ".mp4";
			
			string outputVideoPath = outputVideoAbsPath + outputVideoname;
			
			cv::glob(outputVideoAbsPath, filenames);//��ȡ·���������ļ���
			
			
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
						string img_path = "D:/JLUVision_Demos/picture" + to_string(i) + ".bmp";   //·����Ҫ����
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
//avi��MP42��mp4��mp4v����Сд���ɣ�,size�Լ��Ķ���Ĳ��ܲ���
