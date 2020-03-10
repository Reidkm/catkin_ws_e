#include <opencv2/opencv.hpp> 
#include <iostream> 
using namespace std; 
using namespace cv; 
int main(int argc,char** argv) 
{ // 1.create  FileStorage 
	string strSettingsFile="/home/reid/catkin_ws_1/src/h_test/src/setting.yaml"; 
	cv::FileStorage fwrite(strSettingsFile.c_str(), cv::FileStorage::WRITE); 
	//2.write data 
	float fx= 100; 
	float fy= 101; 
	fwrite<< "fx" << fx; 
	fwrite<< "fy" << fy; 
	//3.close FileStorage 
	fwrite.release(); 
	return 0; 
}

