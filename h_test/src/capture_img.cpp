#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <sstream>
#include <math.h>
//#include <string>

using namespace cv;
using namespace std;


int main(int argc,char **argv)
{
	VideoCapture cap_L(1);
	VideoCapture cap_R(2);
	
	if(!cap_L.isOpened())
	{
		cout<< "could not open the camera_L"<<endl;
		return -1;
	}
	if(!cap_R.isOpened())
	{
		cout << "could not open the camera_R " <<endl;
		return -1;
	}
	Mat img_R,img_L;
	string imgfile,str;
	int count = 1385030200;
	int k=1;
	for(;;)
	{	
		stringstream ss;
		ss << count;
		ss >> str;
		cap_L >> img_L;
		cap_R >> img_R;
		int height = img_L.rows;
		int width = img_L.cols;
		/*
		if (k==1)
		{
			for(int i = 0; i< height;++i)
			{
				for(int j=0;j<width;++j)
				{
					int id = i*width+j;
					cout << "id is " <<id-width<<endl;
				
				}
			}
			k  = k +1;
		}
		*/
		//cout << "img type is :" << img_L.type() << endl;
		//cout << "img depth is :" << img_L.depth() << endl;
		/*
		for(int i= 0;i<img_L.rows;i++)
		{
			Vec3b*dataR = img_R.ptr<Vec3b>(i);
			Vec3b*dataL = img_L.ptr<Vec3b>(i);
			dataL[320][0] = 0;
			dataL[320][1] = 0;
			dataL[320][2] = 255;
			dataR[320][0] = 0;
			dataR[320][1] = 0;
			dataR[320][2] = 255;
		}
		for(int i= 0;i<img_L.cols;i++)
		{
			Vec3b*dataL = img_L.ptr<Vec3b>(240);
			Vec3b*dataR = img_R.ptr<Vec3b>(240);
			dataL[i][0] = 0;
			dataL[i][1] = 0;
			dataL[i][2] = 255;
			dataR[i][0] = 0;
			dataR[i][1] = 0;
			dataR[i][2] = 255;
		}
		*/
		if(img_L.empty() || img_R.empty() ) break;
		imshow("R_rawRGB",img_R);
		//moveWindow("R_rawRGB", 0, 30);
		imshow("L_rawRGB",img_L);
		//moveWindow("L_rawRGB", 200, 30);

		char key = static_cast<char>(waitKey(1));
		if(key == 27) break;
		if(key=='w'||key=='W')
		{	
			
			imgfile="/home/reid/Desktop/tem_folder/stereo_tem_img/img_L_6m_"+str+".jpg";
			imwrite(imgfile,img_L);
			imgfile = "/home/reid/Desktop/tem_folder/stereo_tem_img/img_R_6m_"+str+".jpg";
			imwrite(imgfile,img_R);
			count++;
			cout << "write image_"+str+" done" << endl;		
		}
	}
	cap_L.release();
	cap_R.release();
	cout<<"finish writing"<<endl;
	return 0;
}
