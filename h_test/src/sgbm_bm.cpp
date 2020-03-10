#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <sstream>
#include <math.h>
#include <opencv2/opencv.hpp> 
#include <cvaux.hpp> 
#include "cvaux.h"
#include "cxcore.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <malloc.h>
#include <stdlib.h> 
#include <stdio.h>
#include <string.h>





using namespace cv;
using namespace std;

Mat rgbImageL,rgbImageR;

int mindisparity = 0;
int SADWindowSize = 11;
int uniquenessRatio =10;
int numDisparities=64;

Ptr<StereoBM> bm = StereoBM::create(16, 9);
Ptr<cv::StereoSGBM> sgbm = StereoSGBM::create(mindisparity, numDisparities, SADWindowSize);


Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat xyz;              //三维坐标 
Rect validROIL;      //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;


void sgbm_stereo(int , void*)
{
	Mat disp,disp8;
        cout << "SADWindowSize is " << SADWindowSize <<endl;
	//sgbm->setBlockSize(2*SADWindowSize+5);     //SAD窗口大小，5~21之间为宜
    //int mindisparity = 0;
	//int ndisparities = 64;  
	//int SADWindowSize = 11; 
    //int P1 = 8 * re_gray_ImageL.channels() * SADWindowSize* SADWindowSize;
	//int P2 = 32 * re_gray_ImageL.channels() * SADWindowSize* SADWindowSize;
	int P1 = 8 * rgbImageR.channels() * SADWindowSize* SADWindowSize;
	int P2 = 32 * rgbImageR.channels() * SADWindowSize* SADWindowSize;
	sgbm->setP1(P1);
	sgbm->setP2(P2);
	sgbm->setPreFilterCap(15);
	sgbm->setUniquenessRatio(uniquenessRatio);
	//sgbm->setNumDisparities(numDisparities*16+16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
	//sgbm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
	sgbm->setSpeckleRange(2);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(cv::StereoSGBM::MODE_HH);
	sgbm->compute(rgbImageL, rgbImageR, disp);
	cout << "disp.cols is :" << disp.cols << endl;
	cout << "disp.rows is :" << disp.rows << endl;
	cout << "disp.depth is :" << disp.depth() << endl;
	cout << "disp.type is :" << disp.type() << endl;
	cout << "disp.channels is :" << disp.channels() << endl;
	//disp.convertTo(disp8, CV_32F, 1.0 / 16);                //除以16得到真实视差值
	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
	normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    	//reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    	//xyz = xyz * 16;
	imshow("disparity_sgbm", disp8U);
	waitKey(0);

}


void bm_stereo(int , void*)
{
	
    //bm->setBlockSize(2*blockSize+5);      // 15   2*blockSize+5    //SAD窗口大小，5~21之间为宜
    bm->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
    bm->setPreFilterSize(9);
    bm->setROI1(validROIL);
    bm->setROI2(validROIR);
    bm->setPreFilterCap(31);
    bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
    bm->setNumDisparities(numDisparities*16+16);// 64     numDisparities*16+16       视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
    bm->setTextureThreshold(10); 
    bm->setUniquenessRatio(uniquenessRatio);// 60     uniquenessRatio   uniquenessRatio主要可以防止误匹配
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);      //4  32 
    bm->setDisp12MaxDiff(-1);
    Mat disp, disp8;
    bm->compute(rgbImageL, rgbImageR, disp);//输入图像必须为灰度图
    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
    //Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);  
    //normalize(disp8, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    imshow("disparity", disp8);


}







int main(int argc ,char*argv[])
{
	Mat rgbImageL = imread("/home/reid/disp_result/im0.png",0);
	Mat rgbImageR = imread("/home/reid/disp_result/im1.png",0);
	
	cout << "rgbImageR.cols is :" << rgbImageR.cols << endl;
	cout << "rgbImageR.rows is :" << rgbImageR.rows << endl;
	cout << "rgbImageR.depth is :" << rgbImageR.depth() << endl;
	cout << "rgbImageR.tpye is :" << rgbImageR.type() << endl;
	cout << "rgbImageR.channels is :" << rgbImageR.channels() << endl;


	const int height = rgbImageR.rows ; 
	const int width  = rgbImageR.cols ; 



	//sgbm_stereo(0,0);
	//imshow("rgbImageL", rgbImageL);
	//imshow("rgbImageR", rgbImageR);
	//waitKey(0);
	Mat disp,disp8;
        cout << "SADWindowSize is " << SADWindowSize <<endl;
	//sgbm->setBlockSize(2*SADWindowSize+5);     //SAD窗口大小，5~21之间为宜
    //int mindisparity = 0;
	//int ndisparities = 64;  
	//int SADWindowSize = 11; 
    //int P1 = 8 * re_gray_ImageL.channels() * SADWindowSize* SADWindowSize;
	//int P2 = 32 * re_gray_ImageL.channels() * SADWindowSize* SADWindowSize;
	int P1 = 8 * rgbImageR.channels() * SADWindowSize* SADWindowSize;
	int P2 = 32 * rgbImageR.channels() * SADWindowSize* SADWindowSize;
	sgbm->setP1(P1);
	sgbm->setP2(P2);
	sgbm->setPreFilterCap(15);
	sgbm->setUniquenessRatio(uniquenessRatio);
	//sgbm->setNumDisparities(numDisparities*16+16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
	//sgbm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
	sgbm->setSpeckleRange(2);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(cv::StereoSGBM::MODE_HH);
	sgbm->compute(rgbImageL, rgbImageR, disp);
	cout << "disp.cols is :" << disp.cols << endl;
	cout << "disp.rows is :" << disp.rows << endl;
	cout << "disp.depth is :" << disp.depth() << endl;
	cout << "disp.type is :" << disp.type() << endl;
	cout << "disp.channels is :" << disp.channels() << endl;
	disp.convertTo(disp8, CV_32F, 1.0 / 16);                //除以16得到真实视差值
	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
	normalize(disp8, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    	//reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    	//xyz = xyz * 16;
	imshow("disparity_sgbm", disp8U);
	waitKey(0);
	return 0;
	
}
