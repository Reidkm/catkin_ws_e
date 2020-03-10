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




int main(int argc, char** argv)
{	
	
	Mat rgbImageL = imread("/home/reid/Desktop/tem_folder/stereo_tem_img/img_L_0.jpg");
	cout << "rgbImageL.cols is :" << rgbImageL.cols << endl;
	cout << "rgbImageL.rows is :" << rgbImageL.rows << endl;
	cout << "rgbImageL.depth is :" << rgbImageL.depth() << endl;
	cout << "rgbImageL.tpye is :" << rgbImageL.type() << endl;
	cout << "rgbImageL.channels is :" << rgbImageL.channels() << endl;
	int height =rgbImageL.rows;
	//int width =rgbImageL.cols*rgbImageL.channels();
	int width =rgbImageL.cols;
	if(rgbImageL.isContinuous())
	{
		cout<< "img is continuous" <<endl;	
		
	}
	Vec3b* data;
	for (int i = 50; i < 60; i++) {	//行循环
		data = rgbImageL.ptr<Vec3b>(i);    //ptr函数可以得到图像任意一行的地址。
		for (int j = 90; j < 110; j++) {	        //列循环
			//data[j+10][0] = 255 ;       //开始处理每个像素 
			//data[j+10][1] = 255 ;
			//data[j+10][2] = 255 ;
			//cout <<i<<"行"<<j <<"列 pixel is " <<static_cast<int>(data[j]) << endl;
			//cout << data[j] << endl;
			cout <<i<<"行"<<j <<"列 pixel is " <<data[j] << endl;
			data[j] = 0 ;
			//cout <<i<<"行"<<j <<"列 pixel is " <<data[j] << endl;
		}
	}
/*
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			//彩色图像获取到单个像素
			int pix_b = (int)rgbImageL.at<Vec3b>(i,j)[0];
			int pix_g = (int)rgbImageL.at<Vec3b>(i,j)[1];	
			int pix_r = (int)rgbImageL.at<Vec3b>(i,j)[2];
			cout << pix_b <<"," << pix_g <<"," << pix_r <<","<<";";
			//获取到RGB分量的值。
			//uchar B = pix[0];
			//uchar G = pix[1];
			//uchar R = pix[2];
			//或者使用下面的方法
			//uchar B = img.at<Vec3b>(i, j)[0];
			//uchar G = img.at<Vec3b>(i, j)[1];
			//uchar G = img.at<Vec3b>(i, j)[2];
			//计算灰度值，然后赋值给灰度图中的像素
			//grayImg.at<uchar>(i,j) = R * 0.299 + G * 0.587 + B * 0.114;//灰度著名心理学公式
		}
	}

*/

	imshow("rgbImageL",rgbImageL);
	waitKey(0);
	return 0;
}
	





	//cout << "rgbImageR.cols is :" << rgbImageR.cols << endl;
	//cout << "rgbImageR.rows is :" << rgbImageR.rows << endl;
	//cout << "rgbImageR.depth is :" << rgbImageR.depth() << endl;
	//cout << "rgbImageR.tpye is :" << rgbImageR.type() << endl;
	//cout << "rgbImageR.channels is :" << rgbImageR.channels() << endl;
	
	/*
		for(int row=0; row < height; row++)
		{
        		const uchar *ptr = img.ptr(row);
			for(int col=0; col < width; col++)
			{
            		// const uchar *uc_pixel = ptr;
            		// int a = uc_pixel[0];
            // int b = uc_pixel[1];
            // int c = uc_pixel[2];
            // 不使用中间指针
            			//int a = ptr[0];
				//int b = ptr[1];
				//int c = ptr[2];
            			//sum += a + b + c;
            			//ptr += 3;
				cout<< ptr[
			}
		}
		
		*/
		/*
		for (int i = 0; i < height; i++)
		{
			Vec3b *p = rgbImageR.ptr<Vec3b>(i);
			for (int j = 0; j < width; j++)
			{
			//取出图片第i行第j列的像素
				
				cout << p[j][0]<< p[j][1] << p[j][2] << endls;
                        //当为单通道图片时，直接用p[0]访问
				
				//if (i == j)  //当i=j时 将像素赋值为红色
				//{
				//	p[0] = 0;
				//	p[1] = 0;
				//	p[2] = 255;
				//}
				
			}
		}
			*/
		
		
		/*
		remap(rgbImageL, rectifyImageL, mapLx, mapLy, CV_INTER_LINEAR);
		remap(rgbImageR, rectifyImageR, mapRx, mapRy, CV_INTER_LINEAR);
		cout << "rectifyImageR.cols is :" << rgbImageR.cols << endl;
		cout << "rectifyImageR.rows is :" << rgbImageR.rows << endl;
		cout << "rectifyImageR.depth is :" << rgbImageR.depth() << endl;
		cout << "rectifyImageR.type is :" << rgbImageR.type() << endl;
		cout << "rectifyImageR.channels is :" << rgbImageR.channels() << endl;


	
		//imshow("L_RECTIFIED",rectifyImageL);
		//imshow("R_RECTIFIED",rectifyImageR);
		enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
		int k =( rgbImageR.cols / 8) + 15;
		cout << "k is :" << k <<endl;
     		int numberOfDisparities = ((rgbImageR.cols / 8) + 15) & -16;
		cout << "numberOfDisparities is :" << numberOfDisparities <<endl;
    		cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
    		sgbm->setPreFilterCap(63);
    		int SADWindowSize = 9;
    		int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    		sgbm->setBlockSize(SADWindowSize);
    		int cn = rgbImageL.channels();
    		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    		sgbm->setMinDisparity(0);
    		sgbm->setNumDisparities(numberOfDisparities);
    		sgbm->setUniquenessRatio(10);
    		sgbm->setSpeckleWindowSize(100);
    		sgbm->setSpeckleRange(32);
    		sgbm->setDisp12MaxDiff(1);

    		int alg = STEREO_SGBM;
    		if (alg == STEREO_HH)
        		sgbm->setMode(cv::StereoSGBM::MODE_HH);
    		else if (alg == STEREO_SGBM)
        		sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    		else if (alg == STEREO_3WAY)
        		sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
		Mat disp, disp8;
    		sgbm->compute(rectifyImageL, rectifyImageR, disp);
		cout << "disp.cols is :" << disp.cols << endl;
		cout << "disp.rows is :" << disp.rows << endl;
		cout << "disp.depth is :" << disp.depth() << endl;
		cout << "disp.type is :" << disp.type() << endl;
		cout << "disp.channels is :" << disp.channels() << endl;
		//imshow("disp",disp);
		//disp.convertTo(disp8, CV_32F, 1.0 / 16);                //除以16得到真实视差值
		Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
		normalize(disp, disp8U, 0, 255, NORM_MINMAX,CV_8UC1);
		//disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
		//disp.convertTo(dispU, CV_32F,255 / ((numDisparities * 16 + 16)*16.));
		//Mat disp;
		imshow("disp_normalized",disp8U);
		//Mat depth_fill;
		//disp8U.convertTo(depth_fill, CV_32F, 1, 0);
    		insertDepth32f(disp8U);
    		imshow("disp_filled", disp8U);
		//Mat depth(disp8U.size(),CV_16U);
    		//disp2Depth(disp8U,depth,cameraMatrixL);
   		//imshow("depth_img",depth);
    		//Mat depth_fill;
    		//depth.convertTo(depth_fill, CV_32F, 1, 0);
    		//insertDepth32f(depth_fill);
    		//imshow("disparity_filled", depth_fill);










		waitKey(0);

	
	}
	*/






