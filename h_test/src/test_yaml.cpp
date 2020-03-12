#include <opencv2/opencv.hpp> 
#include <iostream>
#include <sstream>
#include <string>

using namespace std; 
using namespace cv;


int main(int argc,char* argv[]) 
{ 
	
	
	// 1.create  FileStorage 
	string strSettingsFile="/home/reid/catkin_ws/src/h_test/config/test_yaml.yaml";

	//read a yaml file
	cv::FileStorage fs_read(strSettingsFile, cv::FileStorage::READ);

	// first method:use (type) operator on FIleNode
	int frame_count = (int)fs_read["frameCount"];

	std::string date;
  //second method:use FileNode::operator >>
	fs_read["calibrationDate"] >> date;


	//std::vecotr<int> image_size;
	//cv::Size image_size;
	int rows;
	fs_read["imageSize"] >> image_size;  
	//cv::Mat camera_matrix, distort_coefficient;
	//fs_read["cameraMatrix"] >> camera_matrix;
	//fs_read["distCoeffs"] >> distort_coefficient;
	//fs_read["distCoeffs"]["rows"] >> rows;

	cout << "image_size is " << image_size << "\n"
		 << "camera_matrix is " << camera_matrix << "\n"
		 << "distort_coefficient is " << distort_coefficient << "\n"
		 <<  "rows is " << rows << endl;

	//std::vector<Point2f> image_point_vector;
	FileNode n = fs_read["ponitlist"]["point"]; 
	FileNodeIterator it = n.begin(), it_end = n.end();  //遍历节点
	for (; it != it_end; ++it) {
  		cout << *it << '\n';
	}
	//Point2f img_point;
	//fs_read["ponitlist"]["point"][0] >> img_point;
	//cout << "img_point is " << img_point << endl;
	//cv::FileStorage fwrite(strSettingsFile.c_str(), cv::FileStorage::WRITE); 
	//2.write data 
	//float fx= 100; 
	//float fy= 101; 
	//fwrite<< "fx" << fx; 
	//fwrite<< "fy" << fy; 
	//3.close FileStorage 
	//fwrite.release(); 
	return 0; 
}

