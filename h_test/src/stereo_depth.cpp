#include <opencv2/opencv.hpp>   
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/calib3d/calib3d.hpp>      
#include <opencv2/legacy/legacy.hpp>  
#include "time.h"

using namespace cv;

void main()
{
    IplImage* img_r, *img_l;                //定义两个图像指针
    img_r = cvLoadImage("/home/reid/catkin_ws/src/h_test/image/img_R_0.jpg", 0);   //图像指针初始化
    img_l = cvLoadImage("/home/reid/catkin_ws/src/h_test/image/img_R_0.jpg", 0);   //图像指针初始化

    cvShowImage("左边图像", img_l);//源图像显示
    cvShowImage("右边图像", img_r);//源图像显示

    CvMat* norm_disparity = cvCreateMat(img_l->height, img_l->width, CV_8U);
    long time = clock();

    //BM算法  
    //CvMat* disparity = cvCreateMat(img_l->height, img_l->width, CV_32FC1);
    //CvStereoBMState* BMState = cvCreateStereoBMState();   
    //BMState->SADWindowSize = 17;//搜索窗口大小,细腻程度跟这个数值有关
    //BMState->minDisparity = 0;//代表匹配搜苏从哪里开始
    //BMState->numberOfDisparities = 16;//表示最大搜索视差数，原来为16
    //BMState->uniquenessRatio = 25;
    //cvFindStereoCorrespondenceBM(img_l, img_r, disparity, BMState);       //校正图像
    //cvNormalize(disparity, norm_disparity, 45, 160, CV_MINMAX, NULL); //图像归一化
    //cvReleaseMat(&disparity);

    //GC算法
    CvMat* disparity_left = cvCreateMat(img_l->height, img_l->width, CV_16S);
    CvMat* disparity_right = cvCreateMat(img_l->height, img_l->width, CV_16S);
    CvStereoGCState* state = cvCreateStereoGCState(16, 2);
    cvFindStereoCorrespondenceGC(img_l,img_r,disparity_left,disparity_right,state,0);
    cvReleaseStereoGCState(&state);
    cvConvertScale(disparity_left, norm_disparity, -16);
    cvReleaseMat(&disparity_left);
    cvReleaseMat(&disparity_right);

    //给深度图像上伪彩色
    Mat tempMat = Mat(norm_disparity, true);
    Mat img_pseudocolor(tempMat.rows, tempMat.cols, CV_8UC3);//构造RGB图像，参数CV_8UC3教程文档里面有讲解  
    int tmp = 0;
    for (int y = 0; y<tempMat.rows; y++)//转为伪彩色图像的具体算法  
    {
        for (int x = 0; x<tempMat.cols; x++)
        {
            tmp = tempMat.at<unsigned char>(y, x);
            img_pseudocolor.at<Vec3b>(y, x)[0] = abs(255 - tmp); //blue  
            img_pseudocolor.at<Vec3b>(y, x)[1] = abs(127 - tmp); //green  
            img_pseudocolor.at<Vec3b>(y, x)[2] = abs(0 - tmp); //red  
        }
    }
    printf("图像分辨率:%d*%d\n", img_l->width,img_l->height);
    printf("双目深度图计算消耗时间:%dms\n", clock() - time);
    imshow("结果", img_pseudocolor);//图像显示
    cvWaitKey(0);

    //释放所有图像指针占内存与矩阵所占内存，关闭窗口
    cvDestroyAllWindows();
    cvReleaseImage(&img_l);
    cvReleaseImage(&img_r);
    cvReleaseMat(&norm_disparity);
}
