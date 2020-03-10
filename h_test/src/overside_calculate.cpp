#include<iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc.hpp> 
#include <math.h>
  
  
using namespace cv;  
using namespace std;   


Mat temp,img,image_normal,converted_image,dstImage;
double minValue, maxValue;
cv::Point   minLoc, maxLoc;
int left_pixel_leg = 1;
int right_pixel_leg = 1;
int mid_pixel_leg ;
int divide_row = 1;
int min_col = 300;
int img_height = 640;
int img_width = 480;

double CalculatePixelSize(double depth_data_overside_point)
{
	double depht_data_in_mm = depth_data_overside_point * 0.33333 ;
	double num_pixel = 0.0002579*depht_data_in_mm*depht_data_in_mm - 0.6012*depht_data_in_mm +451.7;
	double pixel_size = 220.0/num_pixel;
	return pixel_size;
}
void LargestConnecttedComponent(Mat srcImage, Mat &dstImage)
{
    Mat temp;
    Mat labels;
    srcImage.copyTo(temp);

    //1. 标记连通域
    int n_comps = connectedComponents(temp, labels, 4, CV_16U);
    vector<int> histogram_of_labels;
    for (int i = 0; i < n_comps; i++)//初始化labels的个数为0
    {
        histogram_of_labels.push_back(0);
    }

    int rows = labels.rows;
    int cols = labels.cols;
    for (int row = 0; row < rows; row++) //计算每个labels的个数
    {
        for (int col = 0; col < cols; col++)
        {
            histogram_of_labels.at(labels.at<unsigned short>(row, col)) += 1;
        }
    }
    histogram_of_labels.at(0) = 0; //将背景的labels个数设置为0

    //2. 计算最大的连通域labels索引
    int maximum = 0;
    int max_idx = 0;
    for (int i = 0; i < n_comps; i++)
    {
        if (histogram_of_labels.at(i) > maximum)
        {
            maximum = histogram_of_labels.at(i);
            max_idx = i;
        }
    }

    //3. 将最大连通域标记为1
    for (int row = 0; row < rows; row++) 
    {
        for (int col = 0; col < cols; col++)
        {
            if (labels.at<unsigned short>(row, col) == max_idx)
            {
                labels.at<unsigned short>(row, col) = 255;
            }
            else
            {
                labels.at<unsigned short>(row, col) = 0;
            }
        }
    }

    //4. 将图像更改为CV_8U格式
    labels.convertTo(dstImage, CV_8U);
}

void extremeValue(const cv::Mat& inputGray,int& pixel_max ,int& pixel_min)
{
    
	unsigned short grayValue; 
	unsigned short maxValue = 1;
	unsigned short minValue = 4000;
	for (int y = 0; y < inputGray.rows; y++)
		for (int x = 0; x < inputGray.cols; x ++)
		{
            //grayValue = inputGray.at<uchar>(y, x);
			grayValue = inputGray.at<ushort>(y, x);
			maxValue = std::max(maxValue, grayValue);
			pixel_max = (int)maxValue ;
			//cout << "maxValue is: " << max << endl;
			if(grayValue > 100)
			{
				minValue = std::min(minValue, grayValue);
				pixel_min = (int)minValue;
				//std::cout << "minValue is " <<min <<std::endl;

			}
		}
	
	std::cout << "maxValue is " <<  (int)maxValue << std::endl;
	std::cout << "minValue is " <<  (int)minValue << std::endl;
  
}
	
void Eliminatebackground(  Mat& inputGray)
{
	for (int y = 0; y < inputGray.rows; y++)
		for (int x = 0; x < inputGray.cols; x ++)
		{	
			//if(int(inputGray.at<ushort>(y, x)) >=3100 || int(inputGray.at<ushort>(y, x)) <=2500)
			if(int(inputGray.at<ushort>(y, x)) >=3500)
			{
				inputGray.at<ushort>(y, x) = 0;
			}
			if(y >= 550)
			{
				inputGray.at<ushort>(y, x) = 0;
			}
		}	
	
	
}

int minColumnGoods(const Mat& inputGray)
{
	int mincolnum = 200;
	//int colnum = 2 ;
	for (int y = 0; y < 600; y++)
	{
		for (int x = 0; x < 420; x ++)
		{	
			//if(int(inputGray.at<ushort>(y, x)) >=3100 || int(inputGray.at<ushort>(y, x)) <=2500)
			if(inputGray.at<uchar>(y, x) >0)
			{
				//cout << "x is " << x <<"," << "y is " << y <<endl;
				//colnum = x;
				mincolnum = min(x,mincolnum);
				cout << "mincolnum is " << mincolnum << endl; 
				break;
			}
		}	
	}
	return mincolnum;
}



int colPalletPixel( const Mat& inputGray,int& firstcol)
{
	//int maxcolnum = 0;
	//int firstcol ;
	int secondcol;
	int k = 0;
	
	for (int x = 0; x < inputGray.cols; x ++)
	{	
			//if(int(inputGray.at<ushort>(y, x)) >=3100 || int(inputGray.at<ushort>(y, x)) <=2500)
		if(k==0)
		{
			if(int(inputGray.at<ushort>(471, x)) >0)
			{
				firstcol = x;
				//maxcolnum = max(colnum,maxcolnum);
				k = 1;
				continue;
			}
		}
		else
		{
			if(int(inputGray.at<ushort>(471, x)) == 0)
			{
				secondcol = x;
				//maxcolnum = max(colnum,maxcolnum);
				
				break;
			}
		}
		
	}	
	int num_palletlegpixel = secondcol - firstcol;
	cout << "firstcol is " << firstcol << endl;
	cout << "secondcol is " << secondcol << endl;
	return num_palletlegpixel;
}


Mat scaleGray(const Mat& inputGray)
{
    //Mat outputGray(inputGray.size(), CV_8U);
    Mat outputGray(inputGray.size(), CV_16U);
    //unsigned char grayValue, maxValue = 1;
     unsigned short grayValue, maxValue = 1;
    for (int y = 0; y < inputGray.rows; y++)
        for (int x = 0; x < inputGray.cols; x ++)
        {
            //grayValue = inputGray.at<uchar>(y, x);
	    grayValue = inputGray.at<ushort>(y, x);
            maxValue = max(maxValue, grayValue);
		//cout << "maxValue is: " << (int)maxValue << endl;
        }
         
    //float scale = 255.0 / maxValue;   
     float scale = 65535.0 / maxValue;
    for (int y = 0; y < inputGray.rows; y++)
        for (int x = 0; x < inputGray.cols; x ++)
        {
            //outputGray.at<uchar>(y, x) = static_cast<unsigned char>(inputGray.at<uchar>(y, x) * scale + 0.5);
		outputGray.at<ushort>(y, x) = static_cast<unsigned short>(inputGray.at<ushort>(y, x) * scale+0.5 );
        }
 
    return outputGray;
}
Mat gray2rainbow(const Mat& scaledGray)
{
    Mat outputRainbow(scaledGray.size(), CV_8UC3);
    //unsigned char grayValue;
	unsigned short grayValue;
    for (int y = 0; y < scaledGray.rows; y++)
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<ushort>(y, x);
            Vec3b& pixel = outputRainbow.at<Vec3b>(y, x);
            if (grayValue <= 51)
            {
                pixel[0] = 255;
                pixel[1] = grayValue * 5;
                pixel[2] = 0;
            }
            else if (grayValue <= 102)
            {
                grayValue -= 51;
                pixel[0] = 255 - grayValue * 5;
                pixel[1] = 255;
                pixel[2] = 0;
            }
            else if (grayValue <= 153)
            {
                grayValue -= 102;
                pixel[0] = 0;
                pixel[1] = 255;
                pixel[2] = grayValue * 5;
            }
            else if (grayValue <= 204)
            {
                grayValue -= 153;
                pixel[0] = 0;
                pixel[1] = 255 - static_cast<unsigned char>(grayValue * 128.0 / 51 + 0.5);
                pixel[2] = 255;
            }
            else if (grayValue <= 255)
            {
                grayValue -= 204;
                pixel[0] = 0;
                pixel[1] = 127 - static_cast<unsigned char>(grayValue * 127.0 / 51 + 0.5);
                pixel[2] = 255;
            }
        }
 
    return outputRainbow;
}

Mat gray2pseudocolor(const Mat& scaledGray)
{
	unsigned short grayValue ,maxValue;
    Mat outputPseudocolor(scaledGray.size(), CV_8UC3);

	for (int y = 0; y < scaledGray.rows; y++)
	{
		for (int x = 0; x < scaledGray.cols; x ++)
		{
            
			grayValue = scaledGray.at<ushort>(y, x);
			maxValue = max(maxValue, grayValue);
		
		}
	}		

    //unsigned char grayValue;
	//unsigned short grayValue;
    for (int y = 0; y < scaledGray.rows; y++)
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<ushort>(y, x);
            Vec3b& pixel = outputPseudocolor.at<Vec3b>(y, x);
           // pixel[0] = abs(255 - grayValue);
            //pixel[1] = abs(127 - grayValue);
            //pixel[2] = abs(0 - grayValue);


	    	pixel[0] = abs(255 - grayValue*255/maxValue);
            pixel[1] = abs(127 - grayValue*255/maxValue);
            pixel[2] = abs(0 - grayValue*255/maxValue);
        }
 
    return outputPseudocolor;
}
 
void onMouse(int event, int x, int y, int flags, void *param)
{
	Mat *im = reinterpret_cast<Mat*>(param);
	switch (event)   //调度事件
	{
		case EVENT_LBUTTONDOWN:  //鼠标左键按下事件
//显示像素值
		cout << "at(" << x << "," << y << ") value is:" << static_cast<int>(im->at<uchar>(cv::Point (x, y))) << endl;
//使用Mat对象的at方法来获取(x,y)的像素值
		break;
	}
//鼠标函数还可能收到的事件有：EVENT_MOUSEMOVE,EVENT_LBUTTONUP,EVENT_RBUTTONDOWN,EVENT_RBUTTONUP

}



int main(int argc, char** argv)
{
	 
	Mat src = imread("/home/reid/catkin_ws/src_L80H30.png",CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
	std::cout << "src .size" << src.size() << std::endl;
	transpose(src, temp);
	flip(temp,img,1);
	
	cout << "img depth is " << img.depth() << endl;
	cout << "img type is " << img.type() << endl;
	cout << "img row is " << img.rows << endl;
	cout << "img col is " << img.cols << endl;
	
	imshow("img",img);
	
	
    minMaxLoc(img, &minValue, &maxValue, &minLoc, &maxLoc);
    cout << "图像灰度最小值:" << minValue << "\n"
        << "图像灰度最小值的位置:" << minLoc << "\n"
        << "***********************************" << "\n"
        << "图像灰度最大值:" << maxValue << "\n"
        << "图像灰度最大值的位置:" << maxLoc << endl;

	Mat histo_pallet(img.size(), CV_8UC1, Scalar(0));
	Mat histo_goods_row230(img.size(), CV_8UC1, Scalar(0));
	Mat histo_goods_row150(img.size(), CV_8UC1, Scalar(0));
	Mat histo_goods_row60(img.size(), CV_8UC1, Scalar(0));
	for(int i = 0;i < 285; i++){
		const ushort *ptr = img.ptr<ushort>(515);
		const ushort *ptr_1 = img.ptr<ushort>(230);
		const ushort *ptr_150 = img.ptr<ushort>(150);
		const ushort *ptr_60 = img.ptr<ushort>(60);

		//std::cout << "ptr_230 " <<i<< " is "<< (double)ptr_1[i]<< std::endl;
		//std::cout << "ptr_150 " <<i<< " is "<< (double)ptr_150[i]<< std::endl;
		//std::cout << "ptr_60 " <<i<< " is "<< (double)ptr_60[i]<< std::endl;
		line(histo_pallet, cv::Point (i, img_height), cv::Point(i, img_height - (double)(ptr[i]*img_height)/maxValue), cv::Scalar(255),1,8);
		line(histo_goods_row230, cv::Point (i, img_height), cv::Point(i, img_height - (double)(ptr_1[i]*img_height)/maxValue), cv::Scalar(255),1,8);
		line(histo_goods_row150, cv::Point (i, img_height), cv::Point(i, img_height - (double)(ptr_150[i]*img_height)/maxValue), cv::Scalar(255),1,8);
		line(histo_goods_row60, cv::Point (i, img_height), cv::Point(i, img_height - (double)(ptr_60[i]*img_height)/maxValue), cv::Scalar(255),1,8);
		if (((double)ptr[i]-(double)ptr[i+8]) > 500 && left_pixel_leg == 1 && (double)ptr[i]  > 500 )
		{
			left_pixel_leg = i+4;
			continue;
		}
		else  if (((double)ptr[i]-(double)ptr[i+20]) <-500 && right_pixel_leg == 1 && (double)ptr[i]  > 500)
		{
			right_pixel_leg = i+10 ;
            
		}
	}
    
	

	imshow("histo_pallet_515row",histo_pallet);
	imshow("histo_goods_row230",histo_goods_row230);
	imshow("histo_goods_row150",histo_goods_row150);
	imshow("histo_goods_row60",histo_goods_row60);
	cout << "left_pixel_leg is " << left_pixel_leg << endl;
	cout << "right_pixel_leg is " << right_pixel_leg << endl;
	mid_pixel_leg =  (left_pixel_leg+right_pixel_leg)/2 ;
	cout << "mid_pixel_leg is " << mid_pixel_leg << endl;
	const ushort *ptr = img.ptr<ushort>(510);
	std::cout << "ptr[510][mid_pixel_leg] is "  << ptr[mid_pixel_leg]<< std::endl;
	/*
	Mat histo_pallet_goods(img.size(), CV_8UC1, Scalar(0));
	for(int i =300 ; i <= 510 ;i++ )
	{
		const ushort *ptr = img.ptr<ushort>(i);
		const ushort *ptr_1 = img.ptr<ushort>(i+14);
		
		line(histo_pallet_goods, cv::Point(i-100, img_height), cv::Point(i-100, img_height - (double)(ptr[mid_pixel_leg]*img_height)/maxValue), cv::Scalar(255),1,8);
		if (ptr[mid_pixel_leg] -ptr_1[mid_pixel_leg]>280 && divide_row == 1 && ptr[mid_pixel_leg] > 500 && ptr_1[mid_pixel_leg] > 500)
		{
			divide_row = i+7;
		}
	}
	
	cout << "divide_row is " << divide_row << endl;
	imshow("histo_pallet_goods",histo_pallet_goods);
	*/


	for(int i = 0; i < img_height; i++)
	{
		const ushort *ptr = img.ptr<ushort>(i);
		for(int j = 0;j <mid_pixel_leg ;j++ )
		{
			if ((ptr[j]-ptr[j+6])> 600 && ptr[j] > 1000 && ptr[j+6] > 1000)
			{
				//cout << "第" << i << "行" << endl; 
				//std::cout << "ptr["<<j<<"] is " << ptr[j] << std::endl;
				//std::cout << "ptr["<<j+6<<"] is " << ptr[j+6] << std::endl;
				//cout << "i =" << i << "," << "j = " << j << endl; 

				if(j+3 < min_col)
				{
					min_col = j+3 ;
					int row_min_cols = i;
					std::cout << "row_min_cols is "<< row_min_cols << std::endl;
				}
				break;
			}
		}
	}
	
	/*
	Mat histogram_min_cols(src.size(), CV_8UC1, Scalar(0));
	for(int i =0 ; i <= img_height ;i++ )
	{
		const ushort *ptr = img.ptr<ushort>(i);
		
		
		line(histogram_min_cols, cv::Point(i, src.rows), cv::Point(i, src.rows - (double)(ptr[min_col+15]*src.rows)/maxValue), cv::Scalar(255),1,8);
		
	}
	
	imshow("histogram_min_cols",histogram_min_cols);

	*/



	normalize(img, image_normal, 0, 255, NORM_MINMAX,CV_8UC1);
	line(image_normal, cv::Point (0, 514), cv::Point (img_width, 514), Scalar(0), 1, 8);
	line(image_normal, cv::Point (285, 0), cv::Point (285, img_height), Scalar(0), 1, 8);
	line(image_normal, cv::Point (0, 299), cv::Point (img_width, 299), Scalar(0), 1, 8);
	imshow("image_normal",image_normal);

	Eliminatebackground(img);
	img.convertTo(converted_image, CV_8UC1);
	
	LargestConnecttedComponent(converted_image, dstImage);
	cout << "dstImage depth is " << dstImage.depth() << endl;
	cout << "dstImage type is " << dstImage.type() << endl;
	imshow("dstImage",dstImage);

	/*
	for (int y = 0; y < divide_row; y++){
		for (int x = 0; x < mid_pixel_leg; x ++){		
			if(dstImage.at<uchar>(y, x) >0)
            {
				min_col = min(x,min_col);	
				break;
			}
		}	
	}
	*/
	cout << "min_col is " << min_col << endl; 
	
	cout << "total_pixle is " << left_pixel_leg - min_col << endl; 

	waitKey(0);
	return 0;
}
//,CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH
