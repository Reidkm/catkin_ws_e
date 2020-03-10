#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <sstream>
//#include <string>

using namespace cv;
using namespace std;
int main(int argc,char **argv)
{
	VideoCapture cap("rtsp://admin:kiktech2016@192.168.1.64:554/out.h264");
	
	if(!cap.isOpened())
	{
		cout<< "could not open the camera"<<endl;
		return -1;
	}
	Mat img;
	string imgfile;
	int count =1;
	for(;;)
	{	
		stringstream ss;
		string str;
		ss << count;
		ss >> str;
		cap>>img;
		if(img.empty()) break;
		imshow("pinhole_cam",img);
		char key = static_cast<char>(waitKey(1));
		if(key == 27) break;
		if(key=='w'||key=='W')
		{
			imgfile="/home/reid/Desktop/tem_folder/webcamera_undistort/calib_img_ds2_0971/img_q"+str+".jpg";
			imwrite(imgfile,img);
			count++;		
		}
	}
	cout<<"finish writing"<<endl;
	return 0;
}
