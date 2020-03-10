#include<iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc.hpp> 
#include <math.h>
  
  
using namespace cv;  
using namespace std;   

int  mid_pixel_leg;

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
    //Mat outputGray(inputGray.size(), CV_8U);
    //Mat outputGray(inputGray.size(), CV_16U);
    //unsigned char grayValue, maxValue = 1;
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
        /* 
    //float scale = 255.0 / maxValue;   
     float scale = 65535.0 / maxValue;
    for (int y = 0; y < inputGray.rows; y++)
        for (int x = 0; x < inputGray.cols; x ++)
        {
            //outputGray.at<uchar>(y, x) = static_cast<unsigned char>(inputGray.at<uchar>(y, x) * scale + 0.5);
		outputGray.at<ushort>(y, x) = static_cast<unsigned short>(inputGray.at<ushort>(y, x) * scale+0.5 );
        }
 
    //return outputGray;
	*/
}
	
void Eliminatebackground(  Mat& inputGray)
{
	for (int y = 0; y < inputGray.rows; y++)
		for (int x = 0; x < inputGray.cols; x ++)
		{	
			//if(int(inputGray.at<ushort>(y, x)) >=3100 || int(inputGray.at<ushort>(y, x)) <=2500)
			if(int(inputGray.at<ushort>(y, x)) >=3600)
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
/*
void extremeValue(const Mat& inputGray,int& max_pixel ,int& min_pixel)
{
    //Mat outputGray(inputGray.size(), CV_8U);
    //Mat outputGray(inputGray.size(), CV_16U);
    //unsigned char grayValue, maxValue = 1;
	unsigned short grayValue; 
	unsigned short maxValue = 1;
	unsigned short minValue = 4000;
	for (int y = 0; y < inputGray.rows; y++)
	{
		for (int x = 0; x < inputGray.cols; x ++)
		{
            //grayValue = inputGray.at<uchar>(y, x);
			grayValue = inputGray.at<ushort>(y, x);
			maxValue = max(maxValue, grayValue);
			if(grayValue > 200)
			{
				minValue = min(minValue, grayValue);
				

			}
			
		}
	}
	std::cout << "maxValue is " <<  (int)maxValue << std::endl;
	std::cout << "minValue is " <<  (int)minValue << std::endl;
	max_pixel = (int)maxValue;
	min_pixel = (int)minValue;
}
*/

		
			/*
			if(grayValue > maxValue)
			{
				maxValue = grayValue;
			}
			*/
			
			
			
        
        /* 
    //float scale = 255.0 / maxValue;   
     float scale = 65535.0 / maxValue;
    for (int y = 0; y < inputGray.rows; y++)
        for (int x = 0; x < inputGray.cols; x ++)
        {
            //outputGray.at<uchar>(y, x) = static_cast<unsigned char>(inputGray.at<uchar>(y, x) * scale + 0.5);
		outputGray.at<ushort>(y, x) = static_cast<unsigned short>(inputGray.at<ushort>(y, x) * scale+0.5 );
        }
 
    //return outputGray;
	*/


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
		cout << "at(" << x << "," << y << ") value is:" << static_cast<int>(im->at<uchar>(Point(x, y))) << endl;
//使用Mat对象的at方法来获取(x,y)的像素值
		break;
	}
//鼠标函数还可能收到的事件有：EVENT_MOUSEMOVE,EVENT_LBUTTONUP,EVENT_RBUTTONDOWN,EVENT_RBUTTONUP

}



int main(int argc, char** argv)
{
	Mat  gaosrc, graysrc, cadst, dst,image_normal,temp,img,binary_img,converted_image,converted_image_1;
	Mat bgModel,fgModel; 
	//Mat img_normal,dst;
	Mat src = imread("/home/reid/catkin_ws/src_L100H30.png",CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
	transpose(src, temp);
	flip(temp,img,1);
	
	cout << "img depth is " << img.depth() << endl;
	cout << "img type is " << img.type() << endl;
	cout << "img row is " << img.rows << endl;
	cout << "img col is " << img.cols << endl;
	//int max_pixel,min_pixel;
	//extremeValue(img,max_pixel,min_pixel);
	//line(img, Point(109, 0), Point(109, 640), Scalar(65535), 1, 8);
	imshow("img",img);
	

	double minValue, maxValue;

    Point minLoc, maxLoc;

    minMaxLoc(img, &minValue, &maxValue, &minLoc, &maxLoc);

    cout << "图像灰度最小值:" << minValue << "\n"

        << "图像灰度最小值的位置:" << minLoc << "\n"

        << "***********************************" << "\n"

        << "图像灰度最大值:" << maxValue << "\n"

        << "图像灰度最大值的位置:" << maxLoc << endl;

	Mat histo_pallet(img.size(), CV_8UC1, Scalar(0));
	Mat histo_goods(img.size(), CV_8UC1, Scalar(0));
	int left_pixle_leg = 1;
	int right_pixel_leg = 1;
	for(int i = 0;i < 270; i++)
	{
		const ushort *ptr = img.ptr<ushort>(470);
		const ushort *ptr_1 = img.ptr<ushort>(180);
		//cout << "ptr" <<i<< " is "<< (double)ptr_1[i]<< endl;
		line(histo_pallet, Point(i, 640), Point(i, 640 - ((double)ptr[i])*640.0/maxValue), cv::Scalar(255),1,8);
		line(histo_goods, Point(i, 640), Point(i, 640 - ((double)ptr_1[i])*640.0/maxValue), cv::Scalar(255),1,8);
		
		if (((double)ptr[i]-(double)ptr[i+8]) > 790 && left_pixle_leg == 1 && (double)ptr[i]  > 0 )
		{
			left_pixle_leg = i+4;
			continue;
		}
		else  if ( ((double)ptr[i]-(double)ptr[i+16]) <-790 && right_pixel_leg == 1 && (double)ptr[i]  > 0)
		{
			right_pixel_leg = i+8 ;
		}

		/*
		if ((double)ptr[i]  == 0 && left_pixle_leg == 1 )
		{
			continue;
		}
		else if ((double)ptr[i] > 3400 && left_pixle_leg == 1 )
		{
			continue;
		}
		else if(0 < (double)ptr[i] < 3400 && left_pixle_leg == 1)
		{
			left_pixle_leg = i;
		}
		else if ((double)ptr[i] > 3400 && right_pixel_leg == 1 )
		{
			right_pixel_leg = i ;
			//break;
		}
		*/
	}
	imshow("histo_pallet_470row",histo_pallet);
	imshow("histo_goods_180row",histo_goods);
	cout << "left_pixle_leg is " << left_pixle_leg << endl;
	cout << "right_pixel_leg is " << right_pixel_leg << endl;
	int  mid_pixel_leg=  (left_pixle_leg+right_pixel_leg)/2 ;
	cout << "mid_pixel_leg is " << mid_pixel_leg << endl;
	
	int divide_row = 1;
	Mat histo_pallet_goods(img.size(), CV_8UC1, Scalar(0));
	for(int i =0 ; i <= 470 ;i++ )
	{
		const ushort *ptr = img.ptr<ushort>(i);
		const ushort *ptr_1 = img.ptr<ushort>(i+14);
		//cout << "ptr" <<i<< " is "<< (double)ptr[mid_pixel_leg]<< endl;
		line(histo_pallet_goods, Point(i, 640), Point(i, 640 - ((double)ptr[mid_pixel_leg])*640.0/maxValue), cv::Scalar(255),1,8);
		if (ptr[mid_pixel_leg] -ptr_1[mid_pixel_leg]>280 && divide_row == 1)
		{
			divide_row = i+7;
		}
	}
	
	cout << "divide_row is " << divide_row << endl;
	imshow("histo_pallet_goods",histo_pallet_goods);
	
	/*
	for(int i = 0; i <divide_row; i++)
	{
		
		const ushort *ptr = img.ptr<ushort>(i);
		for(int j = 0;j <mid_pixel_leg ;j++ )
		{
			if ((ptr[j]-ptr[j+10])> 870 && ptr[j] > 0 && ptr[j+10] > 0)
			{
				cout << "i =" << i << "," << "j = " << j << endl; 
				min_col = std::min(j+5,min_col);
				break;
			}
		}
	}
	*/
	
		/*
		for(int j = mid_pixel_leg; j > 0; j-- )
		{
			//cout << ptr[j]-ptr[j-10] << endl;
			if(std::abs(ptr[j]-ptr[j-10])> 870 && ptr[j] > 0 && ptr[j-10])
			{
				//cout << "!!!!!!!!!!!!!!" << endl;
				min_col = std::min(j-5,min_col);
				continue;	
			}
			
		}
		*/
	
	
	



	normalize(img, image_normal, 0, 255, NORM_MINMAX,CV_8UC1);
	line(image_normal, Point(0, 470), Point(480, 470), Scalar(0), 1, 8);
	line(image_normal, Point(270, 0), Point(270, 640), Scalar(0), 1, 8);
	imshow("image_normal",image_normal);
	//img.convertTo(converted_image_1, CV_8UC1,255.0/maxValue);
	//imshow("converted_image_1",converted_image_1);
	//std::cout << "255.0/max_pixel is "<< 255.0/max_pixel << std ::endl;
	//std::cout << "255/max_pixel is "<< 255/max_pixel << std ::endl;
	Eliminatebackground(img);
	img.convertTo(converted_image, CV_8UC1);
	Mat dstImage;
	LargestConnecttedComponent(converted_image, dstImage);
	cout << "dstImage depth is " << dstImage.depth() << endl;
	cout << "dstImage type is " << dstImage.type() << endl;
	imshow("dstImage",dstImage);

	int min_col = 200;
	for (int y = 0; y < divide_row; y++)
	{
		for (int x = 0; x < mid_pixel_leg; x ++)
		{	
			//if(int(inputGray.at<ushort>(y, x)) >=3100 || int(inputGray.at<ushort>(y, x)) <=2500)
			if(dstImage.at<uchar>(y, x) >0)
			{
				//cout << "x is " << x <<"," << "y is " << y <<endl;
				//colnum = x;
				min_col = min(x,min_col);
				
				break;
			}
		}	
	}
	cout << "min_col is " << min_col << endl; 
	cout << "total_pixle is " << left_pixle_leg - min_col << endl; 



	//int min_col =minColumnGoods(dstImage);
	//cout << "min_col is " << min_col<< endl;
	//cv::threshold(img, binary_img, 3400, 65535, CV_THRESH_BINARY_INV);
	//imshow("binary_img",binary_img);
	//int pixel_max,pixel_min;
	//extremeValue(img,pixel_max,pixel_min);
	//std::cout << " pixel_max is " << pixel_max <<std::endl;
	//std::cout << " pixel_min is " << pixel_min <<std::endl;
	
	//Mat scaledGray = scaleGray(img);
	
	//line(image_normal, Point(0, 400), Point(480, 400), Scalar(0), 1, 8);
	//Mat gray2rainbow = gray2rainbow(image_normal);
	//imshow("gray2rainbow",gray2rainbow);
	
	//Mat pseudocolor = gray2pseudocolor(image_normal);
	
	//extremeValue(img);
	//const ushort *ptr = img.ptr<ushort>(321);
	//cout << "pixel is " << (int)ptr[240] << endl;
	//cout << "pixel is " << ((int)ptr[240])*0.3333 << endl;
	//line(img, Point(0, 321), Point(480, 321), Scalar(255), 1, 8);
	//const short *ptr = img.ptr<short>(195);
	//cout << "pixel is " << (int)ptr[3] << endl;
	//line(img, Point(0, 471), Point(480, 471), Scalar(65535), 1, 8);
	//line(img, Point(50, 0), Point(50, 640), Scalar(65535), 1, 8);
	/*
	for(int i=0; i < 480 ; i++)
	{
		const ushort *ptr = img.ptr<ushort>(471);
		cout << "pixel "<< i<<" is " << (int)ptr[i] << endl;
		 
	}
	*/
	//Mat gray2pseudocolor=gray2pseudocolor(image_normal);
	
	//normalize(img, image_normal, 0, 65535, NORM_MINMAX,CV_16UC1);
	
	//imshow("pseudocolor",pseudocolor);
	//Mat scaledGray = scaleGray(img);
	//Eliminatebackground(img);
	//imshow("img_elimate",img);
	//int minnumgoods = minColumnGoods(img);
	int firstcol;
	int num_palletlegpixel;
        double length_palletleg = 50;
	//num_palletlegpixel=colPalletPixel(img,firstcol);
	//cout << "num_palletlegpixel in 100cm is " << num_palletlegpixel << endl;
	double unit = length_palletleg/(double)num_palletlegpixel;
	//cout << "unitis " << unit << endl;
	//int maxnumpallet = maxColumnPallet(img);
	//cout << "maxnumgoods is " << minnumgoods << endl;
	//cout << "maxnumpallet is " << maxnumpallet << endl;
	//cout << "overside pixel is " << firstcol-minnumgoods << endl;
	//cout << "overside length is " << (firstcol-minnumgoods)* unit<<"mm"<< endl;
		
	
	//normalize(img, image_normal, 0, 65535, NORM_MINMAX,CV_16UC1);
	
	//const short *ptr_1 = image_normal.ptr<short>(321);
	//cout << "pixel is " << (int)ptr_1[240] << endl;
	//cout << "pixel is " << ((int)ptr_1[240])*0.3333 << endl;
	//line(image_normal, Point(0, 321), Point(480, 321), Scalar(255), 1, 8);
	//line(image_normal, Point(240, 0), Point(240, 640), Scalar(255), 1, 8);
	//line(image_normal, Point(162, 0), Point(162, 640), Scalar(255), 1, 8);
	//line(image_normal, Point(0, 421), Point(480, 421), Scalar(255), 1, 8);
	//line(image_normal, Point(0, 471), Point(480, 471), Scalar(255), 1, 8);
	//imshow("image_normal",image_normal);
	
	//line(rectifyImageR, Point(313, 434), Point(313, 474), Scalar(0, 0, 255), 1, 8);
	//changeGray(img);
	//Mat scaledGray = scaleGray(img);
	//Mat pseudocolor = gray2pseudocolor(scaledGray);
	//Mat rainbow = gray2rainbow(scaledGray);
	//Mat img = imread("/home/reid/catkin_ws/depth_cut.png",CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
	//Mat img = imread("/home/reid/.ros/temp/frame0000.png");
	//normalize(img, img_normal, 0, 65535, NORM_MINMAX,CV_16UC1);
	//cout << "img depth is " << img.depth() << endl;
	//cout << "img type is " << img.type() << endl;
	//cout << "img row is " << img.rows << endl;
	//cout << "img col is " << img.cols << endl;


	//const short *ptr = img.ptr<short>(241);
	//cout << "pixel is " << (int)ptr[320] << endl;
	//cout << "pixel is " << ((int)ptr[320])*0.3333 << endl;
	//Vec3b *p = rgbImageR.ptr<Vec3b>(i);
	//img.row(90).setTo(0) ;//设定第i行数据
	/*
	for(int i=0;i<240;i++)
	{
		img.row(i).setTo(0);	
	}
	*/
	//normalize(img, image_normal, 0, 65535, NORM_MINMAX,CV_16UC1);
	//normalize(img, image_normal, 0, 255, NORM_MINMAX,CV_8UC3);
	//Mat pseudocolor = gray2pseudocolor(image_normal);
	//cv::Rect rectangle(0,0,pseudocolor.cols-120,pseudocolor.rows-70);
	//cv::Mat result;
	//cv::grabCut(pseudocolor,result,rectangle,bgModel,fgModel, 1,cv::GC_INIT_WITH_RECT); 
	//cv::compare(result,cv::GC_PR_FGD,result,cv::CMP_EQ);
	//cv::Mat foreground(pseudocolor.size(),CV_8UC3,cv::Scalar(255,255,255));
	//pseudocolor.copyTo(foreground,result); 
	//transpose(foreground, temp);
	//flip(temp,dst,1);
	//imshow("foreground",dst);
	//GaussianBlur(rainbow, gaosrc, Size(3, 3), 0);
	//cvtColor(gaosrc, graysrc, CV_RGB2GRAY);
	//setMouseCallback("rainbow", onMouse, reinterpret_cast<void *>(&rainbow));
	//Mat im_color,temp;
	//applyColorMap(img, im_color, COLORMAP_JET);
	//LUT(img, lut, im_color);
	//imshow("scaledGray",scaledGray);
	//imshow("pseudocolor",pseudocolor);
	//Canny(image_normal, dst, 100, 200);
	//transpose(img, temp);
	//flip(temp,dst,1);
	//cout << "img depth is " << dst.depth() << endl;
	//cout << "img type is " << dst.type() << endl;
	//cout << "img row is " << dst.rows << endl;
	//cout << "img col is " << dst.cols << endl;
	//imshow("dst",dst);
	//imshow("image_normal",image_normal);
	//imshow("dst",dst);
	//cv::imwrite("depth_cut.png", img);
	//imshow("img_normal",img_normal);
	//imshow("img",img);
	waitKey(0);


	return 0;
}
//,CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH
