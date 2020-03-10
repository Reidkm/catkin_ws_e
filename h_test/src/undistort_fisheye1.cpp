#include <iostream> 
#include <sstream> 
#include <time.h>
#include <stdio.h> 
#include <fstream> 
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/calib3d/calib3d.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <string>
#include <opencv2/opencv.hpp>
#include <fstream> 

using namespace std; 
using namespace cv; 

int main(int argc,char** argv)
{
	
	string yaml_path = "/home/reid/catkin_ws_1/src/h_test/src/setting.yaml";
	//cv::FileStorage fread(yaml_path.c_str(),cv::FileStorage::READ);
	FileStorage fs( yaml_path.c_str(), FileStorage::READ);
	//float mtx1[] = {1.0267465313163195e+03, 0., 9.2877726677438488e+02, 0.,1.0343895583142830e+03, 5.2760129595866567e+02, 0., 0., 1.};
	//float dist1[] = {5.4510442182540714e-03, -2.6474929866584912e-01,7.1076316949583163e-01, -6.5706501309179788e-01};
	
	Mat mtx(3,3,CV_32FC1);
	Mat dist(1,4,CV_32FC1);
	
	fs ["mtx"] >> mtx;
	fs ["dist"] >> dist;
	
	
	cout << mtx <<endl;
	cout <<dist <<endl;

	Mat DistortImg = cv::imread("/home/reid/Desktop/tem_folder/webcamera_undistort/temp_img_dsipc/img_23_corner.jpg");
	cout << DistortImg.size()<<endl;
	Size image_size = DistortImg.size();
	//cout << image_size.width<<endl;

	Mat UndistortImg,mapx,mapy;
	//Mat new_intrinsic_mat;    //Mat new_intrinsic_mat(3, 3, CV_64FC1, Scalar(0))亦可，注意数据类型;


	//fx,fy变大（小），视场变小（大），裁剪较多（少），但细节清晰（模糊）；很关键，new_intrinsic_mat决定输出的畸变校正图像的范围
	//mtx.copyTo(new_intrinsic_mat);

	//调整输出校正图的视场
	//new_intrinsic_mat.at<double>(0, 0) *= 0.5;      //注意数据类型，非常重要
	//new_intrinsic_mat.at<double>(1, 1) *= 0.4; 


	//调整输出校正图的中心
	//new_intrinsic_mat.at<double>(0, 2) += 0.5 * DistortImg.cols;   
	//new_intrinsic_mat.at<double>(1, 2) += 0.5 * DistortImg.rows;
	//Mat mapx = Mat(image_size, CV_32FC1);
   	//Mat mapy = Mat(image_size, CV_32FC1);
	fisheye::initUndistortRectifyMap(mtx, dist, Mat(), getOptimalNewCameraMatrix(mtx, dist, image_size, 1, image_size, 0), image_size, CV_32FC1, mapx, mapy);
	cout << "img_size:" << image_size <<endl;
	remap(DistortImg, UndistortImg, mapx, mapy, INTER_LINEAR);
	//undistort( DistortImg, UndistortImg, mtx, dist, mtx );
	//cv::fisheye::undistortImage(DistortImg,UndistortImg,mtx,dist,mtx); 
       //最后一个camera_matrix必须写上  Ref:http://answers.opencv.org/question/64614/fisheyeundistortimage-doesnt-work-what-wrong-with-my-code/


	cv::imshow("DistortImg", DistortImg);
	cv::imshow("UndistortImg", UndistortImg);
	//cv::imwrite("feModelUndistortFore.jpg", UndistortImg);
	cv::waitKey(0);


	return 0;





}
