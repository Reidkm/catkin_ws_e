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


enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
const int imageWidth = 640;                             //摄像头的分辨率  
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, re_gray_ImageL;
Mat rgbImageR, re_gray_ImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;      //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;     //映射表  
Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat xyz;              //三维坐标 

Point origin;         //鼠标按下的起始点
Rect selection;      //定义矩形选框
bool selectObject = false;    //是否选择对象

int blockSize = 0, uniquenessRatio =0, numDisparities=0;
Ptr<StereoBM> bm = StereoBM::create(16, 9);
Ptr<cv::StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);



Mat cameraMatrixL = (Mat_<double>(3, 3) << 451.397948, 0.000000, 343.062759, 0.000000, 452.928396, 245.413853, 0.000000, 0.000000, 1.000000);
Mat distCoeffL = (Mat_<double>(5, 1) << -0.037532, 0.078008, 0.011118, 0.007897, 0.000000);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 449.200066, 0.000000, 339.672170, 0.000000, 450.243455, 253.448507, 0.000000, 0.000000, 1.000000);
Mat distCoeffR = (Mat_<double>(5, 1) << -0.051695, 0.079776, 0.008636, 0.002605, 0.000000);

Mat T = (Mat_<double>(3, 1) << 204.081451656473,0,0);//T平移向量
//Mat rec = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//rec旋转向量
Mat R = (Mat_<double>(3, 3) << 0.998453237208844,0.0279672732036317,0.0480516881777589, 
                                -0.0298684171587964,0.998780442220761,0.0393129227319578,
                                -0.0468936111171582,-0.0406873428335421,0.998070903979040);    //R 旋转矩阵

Mat ros_Pl = (Mat_<double>(3, 3) << 499.706886, 0.000000, 302.546371, 0.000000, 499.706886, 253.744099, 0.000000, 0.000000, 1.000000);
Mat ros_Rl = (Mat_<double>(3, 3) << 0.996106, -0.007852, 0.087808, 0.008688, 0.999921 ,-0.009136, -0.087730 ,0.009863, 0.996095);
Mat ros_Rr = (Mat_<double>(3, 3) << 0.999278, 0.011858, 0.036098, -0.012201, 0.999882 ,0.009290, -0.035983 ,-0.009724, 0.999305);
                                
void insertDepth32f(cv::Mat& depth)
{
    
    const int width = depth.cols;
    const int height = depth.rows;
    uchar* data = (uchar*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    // 积分区间
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = max(0, left);
                right = min(right, width - 1);
                top = max(0, top);
                bot = min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = uchar(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(depth, depth, cv::Size(s, s), 0, 0);
    }
}


void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K)
{
    int type = dispMap.type();

    float fx = 463.203457;
    float fy = K.at<float>(1, 1);
    float cx = K.at<float>(0, 2);
    float cy = K.at<float>(1, 2);
    float baseline = 204; //基线距离500mm

    if (type == CV_8U)
    {
        const float PI = 3.14159265358;
        int height = dispMap.rows;
        int width = dispMap.cols;
        
        uchar* dispData = (uchar*)dispMap.data;
        ushort* depthData = (ushort*)depthMap.data;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                int id = i*width + j;
                if (!dispData[id])  
                {
                    continue;  //防止0除
                }
                depthData[id] = ushort( (float)fx *baseline / ((float)dispData[id]) );
            }
        }
    }
    else
    {
        cout << "please confirm dispImg's type!" << endl;
        cv::waitKey(0);
    }
}


void stereo_match(int,void*)
{
    bm->setBlockSize(2*blockSize+5);      // 15   2*blockSize+5    //SAD窗口大小，5~21之间为宜
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
    bm->compute(re_gray_ImageL, re_gray_ImageR, disp);//输入图像必须为灰度图
    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
    //Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);  
    //normalize(disp8, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    imshow("disparity", disp8);

}
void stereo_match_sgbm(int,void*)
{   
	Mat disp,disp8;
	int SADWindowSize = blockSize;
    //cout << "SADWindowSize is " << SADWindowSize <<endl;
	sgbm->setBlockSize(2*SADWindowSize+5);     //SAD窗口大小，5~21之间为宜
    //int mindisparity = 0;
	//int ndisparities = 64;  
	//int SADWindowSize = 11; 
    //int P1 = 8 * re_gray_ImageL.channels() * SADWindowSize* SADWindowSize;
	//int P2 = 32 * re_gray_ImageL.channels() * SADWindowSize* SADWindowSize;
	int P1 = 8 * re_gray_ImageL.channels() * SADWindowSize* SADWindowSize;
	int P2 = 32 * re_gray_ImageL.channels() * SADWindowSize* SADWindowSize;
	sgbm->setP1(P1);
	sgbm->setP2(P2);
	sgbm->setPreFilterCap(15);
	//sgbm->setUniquenessRatio(10);
	sgbm->setNumDisparities(numDisparities*16+16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
	sgbm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
	sgbm->setSpeckleRange(32);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setDisp12MaxDiff(-1);
	sgbm->setMode(cv::StereoSGBM::MODE_HH);
	sgbm->compute(re_gray_ImageL, re_gray_ImageR, disp);
	disp.convertTo(disp8, CV_32F, 1.0 / 16);                //除以16得到真实视差值
	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
	normalize(disp8, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
	imshow("disparity_sgbm", disp8U);
    

}

void sgbm_depth(int,void*)
{   
    
    Mat disp ,disp8;
    int numberOfDisparities = ((imageSize.width / 8) + 15) & -16;
    sgbm->setPreFilterCap(32);
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
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
    sgbm->compute(re_gray_ImageL, re_gray_ImageR, disp);
    disp.convertTo(disp8, CV_8U, 255 / ((numberOfDisparities * 16 + 16)*16.));
    imshow("disparity_2", disp8);
    Mat depth(disp8.size(),CV_16U);
    disp2Depth(disp8,depth,cameraMatrixL);
    imshow("depth_img",depth);
    Mat depth_fill;
    depth.convertTo(depth_fill, CV_32F, 1, 0);
    insertDepth32f(depth_fill);
    imshow("disparity_filled", depth_fill);

}



/*****描述：鼠标操作回调*****/
static void onMouse(int event, int x, int y, int, void*)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event)
    {
    case EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        cout << origin <<"in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
        break;
    case EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
        break;
    }
}



int main(int argc, char** argv)
{	
	initUndistortRectifyMap(cameraMatrixL, distCoeffL, ros_Rl, ros_Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, ros_Rr, ros_Pl, imageSize, CV_32FC1, mapRx, mapRy);
	VideoCapture cap_L(1);
	VideoCapture cap_R(2);
	//Mat rgbImageL = imread("/home/reid/disp_result/img_L_0.jpg");
	//Mat rgbImageR = imread("/home/reid/disp_result/img_R_0.jpg");
	//int height = rgbImageL.rows;
	//int width = rgbImageL.cols;

	cout << "rgbImageR.cols is :" << rgbImageR.cols << endl;
	cout << "rgbImageR.rows is :" << rgbImageR.rows << endl;
	cout << "rgbImageR.depth is :" << rgbImageR.depth() << endl;
	cout << "rgbImageR.tpye is :" << rgbImageR.type() << endl;
	cout << "rgbImageR.channels is :" << rgbImageR.channels() << endl;
	//Mat rgbImageL,rgbImageR;
	for(;;)
	{
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
		cap_L >> rgbImageL;
		cap_R >> rgbImageR;
		//rgbImageL.row(90).setTo(Scalar(0 , 0 , 255)) ;//设定第i行数据
		//rgbImageL.col(240).setTo(Scalar(0 , 0 , 255)) ;//设定第i列数据
		imshow("rgb_L",rgbImageL);
		imshow("rgb_R",rgbImageR);
		int height = rgbImageL.rows;
		int width = rgbImageL.cols;
		
		remap(rgbImageL, rectifyImageL, mapLx, mapLy, CV_INTER_LINEAR);
		remap(rgbImageR, rectifyImageR, mapRx, mapRy, CV_INTER_LINEAR);
		//cout << "rgbImageR.cols is :" << rgbImageR.cols << endl;
		//cout << "rgbImageR.rows is :" << rgbImageR.rows << endl;
		//cout << "rgbImageR.depth is :" << rgbImageR.depth() << endl;
		//cout << "rgbImageR.type is :" << rgbImageR.type() << endl;
		//cout << "rgbImageR.channels is :" << rgbImageR.channels() << endl;

		line(rectifyImageL, Point(220, 90), Point(260, 90), Scalar(0, 0, 255), 1, 8);
		line(rectifyImageL, Point(240, 70), Point(240, 110), Scalar(0, 0, 255), 1, 8);
		putText(rectifyImageL,"(240,90)",Point(247,85),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);

		line(rectifyImageR, Point(168, 90), Point(208, 90), Scalar(0, 0, 255), 1, 8);
		line(rectifyImageR, Point(188, 70), Point(188, 110), Scalar(0, 0, 255), 1, 8);
		putText(rectifyImageR,"(188,90)",Point(193,85),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);
		

		//line(rectifyImageL, Point(226, 454), Point(351, 454), Scalar(0, 0, 255), 1, 8);
		//line(rectifyImageL, Point(331, 434), Point(331, 474), Scalar(0, 0, 255), 1, 8);
		line(rectifyImageL, Point(313, 454), Point(353, 454), Scalar(0, 0, 255), 1, 8);
		line(rectifyImageL, Point(333, 434), Point(333, 474), Scalar(0, 0, 255), 1, 8);
		putText(rectifyImageL,"(333,454)",Point(339,448),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);

		line(rectifyImageR, Point(293, 454), Point(333, 454), Scalar(0, 0, 255), 1, 8);
		line(rectifyImageR, Point(313, 434), Point(313, 474), Scalar(0, 0, 255), 1, 8);
		putText(rectifyImageR,"(313,454)",Point(317,449),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);

		line(rectifyImageL, Point(283, 460), Point(323, 460), Scalar(0, 0, 255), 1, 8);
		line(rectifyImageL, Point(300, 440), Point(300, 480), Scalar(0, 0, 255), 1, 8);
		putText(rectifyImageL,"(300,460)",Point(200,455),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);
		
		line(rectifyImageR, Point(263, 460), Point(303, 460), Scalar(0, 0, 255), 1, 8);
		line(rectifyImageR, Point(282, 440), Point(282, 480), Scalar(0, 0, 255), 1, 8);
		putText(rectifyImageR,"(282,460)",Point(200,455),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);
		//rectifyImageL.row(90).setTo(Scalar(0 , 0 , 255)) ;//设定第i行数据
		//rectifyImageL.col(240).setTo(Scalar(0 , 0 , 255)) ;//设定第i列数据
		//rectifyImageR.col(188).setTo(Scalar(0 , 0 , 255)) ;//设定第i列数据
		//rectifyImageR.row(90).setTo(Scalar(0 , 0 , 255)) ;//设定第i列数据
		//rectifyImageL.col(331).setTo(Scalar(0 , 0 , 255)) ;//设定第i列数据
		//rectifyImageL.row(454).setTo(Scalar(0 , 0 , 255)) ;//设定第i列数据
		
		//putText(rectifyImageL,"(246,454)",Point(252,450),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);
		//putText(rectifyImageR,"(188,90)",Point(193,85),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,8);
		imshow("L_RECTIFIED",rectifyImageL);
		imshow("R_RECTIFIED",rectifyImageR);
		
		enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
		int k =( rgbImageR.cols / 8) + 15;
		//cout << "k is :" << k <<endl;
     		int numberOfDisparities = ((rgbImageR.cols / 8) + 15) & -16;
		//cout << "numberOfDisparities is :" << numberOfDisparities <<endl;
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
		cout << "[90,240] value is :" << disp.at<short>(90, 240)/16<< endl;
		cout << "[454,333] value is :" << disp.at<short>(454, 333)/16<< endl;
		//cout << "[449,255] value is :" << disp.at<short>(449, 255)/16<< endl;
		cout << "[460,300] value is :" << disp.at<short>(460, 300)/16<< endl;
		//disp.row(90).setTo(0) ;//设定第i行数据
		//disp.col(240).setTo(0) ;//设定第i列数据
		//disp.row(454).setTo(0) ;//设定第i行数据
		//disp.col(331).setTo(0) ;//设定第i列数据
		
		cout << "disp.cols is :" << disp.cols << endl;
		cout << "disp.rows is :" << disp.rows << endl;
		cout << "disp.depth is :" << disp.depth() << endl;
		cout << "disp.type is :" << disp.type() << endl;
		cout << "disp.channels is :" << disp.channels() << endl;
		//imshow("disp",disp);
		//disp.convertTo(disp8, CV_32F, 1.0 / 16);                //除以16得到真实视差值
		Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
		normalize(disp, disp8U, 0, 255, NORM_MINMAX,CV_8UC1);
		line(disp8U, Point(220, 90), Point(260, 90), 0, 1, 8);
		line(disp8U, Point(240, 70), Point(240, 110), 0, 1, 8);

		line(disp8U, Point(313, 454), Point(353, 454), 255, 1, 8);
		line(disp8U, Point(333, 434), Point(333, 474), 255, 1, 8);

			
		line(disp8U, Point(283, 460), Point(323, 460), 255, 1, 8);
		line(disp8U, Point(300, 440), Point(300, 480), 255, 1, 8);
		//cout << "disp8U.cols is :" << disp8U.cols << endl;
		//cout << "disp8U.rows is :" << disp8U.rows << endl;
		//cout << "disp8U.depth is :" << disp8U.depth() << endl;
		//cout << "disp8U.type is :" << disp8U.type() << endl;
		//cout << "disp8U.channels is :" << disp8U.channels() << endl;
		//disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
		//disp.convertTo(dispU, CV_32F,255 / ((numDisparities * 16 + 16)*16.));
		//Mat disp;
		imshow("disp_normalized",disp8U);
		//Mat depth_fill;
		//disp8U.convertTo(depth_fill, CV_32F, 1, 0);
    		insertDepth32f(disp8U);
    		//imshow("disp_filled", disp8U);
		//Mat depth(disp8U.size(),CV_16U);
    		//disp2Depth(disp8U,depth,cameraMatrixL);
   		//imshow("depth_img",depth);
    		//Mat depth_fill;
    		//depth.convertTo(depth_fill, CV_32F, 1, 0);
    		//insertDepth32f(depth_fill);
    		//imshow("disparity_filled", depth_fill);





		//显示在同一张图上
        	Mat canvas;
        	double sf;
		int w, h;
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);
		canvas.create(h, w * 2, CV_8UC3);   //注意通道


        	//左图像画到画布上
        	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分  
        	resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小  
        	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域    
        	cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
        //rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形  
        //cout << "Painted ImageL" << endl;

        //右图像画到画布上
        	canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分  
        	resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
        	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
        	cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
        //rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
        //cout << "Painted ImageR" << endl;

        //画上对应的线条
		/*
        	for (int i = 0; i < canvas.rows; i += 16)
		{
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
		}
            		*/
        	imshow("rectified", canvas);





		waitKey(1);

	
	}

	return 0;
}
























/******************************/
/*        立体匹配和测距        */
/******************************/

/*
#include <opencv2/opencv.hpp>  
#include <iostream>  

using namespace std;
using namespace cv;

const int imageWidth = 640;                             //摄像头的分辨率  
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy;     //映射表  
Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat xyz;              //三维坐标

Point origin;         //鼠标按下的起始点
Rect selection;      //定义矩形选框
bool selectObject = false;    //是否选择对象

int blockSize = 0, uniquenessRatio =0, numDisparities=0;
Ptr<StereoBM> bm = StereoBM::create(16, 9);
*/
/*
事先标定好的相机的参数
fx 0 cx
0 fy cy
0 0  1
*/

/*

Mat cameraMatrixL = (Mat_<double>(3, 3) << 682.55880, 0, 384.13666,
    0, 682.24569, 311.19558,
    0, 0, 1);
Mat distCoeffL = (Mat_<double>(5, 1) << -0.51614, 0.36098, 0.00523, -0.00225, 0.00000);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 685.03817, 0, 397.39092,
    0, 682.54282, 272.04875,
    0, 0, 1);
Mat distCoeffR = (Mat_<double>(5, 1) << -0.46640, 0.22148, 0.00947, -0.00242, 0.00000);

Mat T = (Mat_<double>(3, 1) << -61.34485, 2.89570, -4.76870);//T平移向量
Mat rec = (Mat_<double>(3, 1) << -0.00306, -0.03207, 0.00206);//rec旋转向量
Mat R;//R 旋转矩阵

*/

/*****立体匹配*****/

/*

void stereo_match(int,void*)
{
    bm->setBlockSize(2*blockSize+5);     //SAD窗口大小，5~21之间为宜
    bm->setROI1(validROIL);
    bm->setROI2(validROIR);
    bm->setPreFilterCap(31);
    bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
    bm->setNumDisparities(numDisparities*16+16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
    bm->setTextureThreshold(10); 
    bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(-1);
    Mat disp, disp8;
    bm->compute(rectifyImageL, rectifyImageR, disp);//输入图像必须为灰度图
    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
    reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    imshow("disparity", disp8);
}


*/

/*****描述：鼠标操作回调*****/

/*
static void onMouse(int event, int x, int y, int, void*)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event)
    {
    case EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        cout << origin <<"in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
        break;
    case EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
        break;
    }
}
*/

/*****主函数*****/


/*

int main()
{
    
    //立体校正
    
    Rodrigues(rec, R); //Rodrigues变换
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
        0, imageSize, &validROIL, &validROIR);
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

   
    //读取图片
    
    rgbImageL = imread("left1.jpg", CV_LOAD_IMAGE_COLOR);
    cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
    rgbImageR = imread("right1.jpg", CV_LOAD_IMAGE_COLOR);
    cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

    imshow("ImageL Before Rectify", grayImageL);
    imshow("ImageR Before Rectify", grayImageR);

    
    //经过remap之后，左右相机的图像已经共面并且行对准了
    
    remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
    remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

    
    //把校正结果显示出来
    
    Mat rgbRectifyImageL, rgbRectifyImageR;
    cvtColor(rectifyImageL, rgbRectifyImageL, CV_GRAY2BGR);  //伪彩色图
    cvtColor(rectifyImageR, rgbRectifyImageR, CV_GRAY2BGR);

    //单独显示
    //rectangle(rgbRectifyImageL, validROIL, Scalar(0, 0, 255), 3, 8);
    //rectangle(rgbRectifyImageR, validROIR, Scalar(0, 0, 255), 3, 8);
    imshow("ImageL After Rectify", rgbRectifyImageL);
    imshow("ImageR After Rectify", rgbRectifyImageR);

    //显示在同一张图上
    Mat canvas;
    double sf;
    int w, h;
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);   //注意通道

    //左图像画到画布上
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分  
    resize(rgbRectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小  
    Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域    
        cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
    //rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形  
    cout << "Painted ImageL" << endl;

    //右图像画到画布上
    canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分  
    resize(rgbRectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
        cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
    //rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
    cout << "Painted ImageR" << endl;

    //画上对应的线条
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    imshow("rectified", canvas);

    
    //立体匹配
    
    namedWindow("disparity", CV_WINDOW_AUTOSIZE);
    // 创建SAD窗口 Trackbar
    createTrackbar("BlockSize:\n", "disparity",&blockSize, 8, stereo_match);
    // 创建视差唯一性百分比窗口 Trackbar
    createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 50, stereo_match);
    // 创建视差窗口 Trackbar
    createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 16, stereo_match);
    //鼠标响应函数setMouseCallback(窗口名称, 鼠标回调函数, 传给回调函数的参数，一般取0)
    setMouseCallback("disparity", onMouse, 0);
    stereo_match(0,0);

    waitKey(0);
    return 0;
    
}
*/





