/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
// Undeprecate CRT functions


#include "h_test/test_m5_openni2_class.h"
//#include "h_test/OniSampleUtilities.h"
#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <istream>
#include <sys/time.h>
#include <ctime>
#include <sstream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>



cv::Mat img,dstImage;
double minValue, maxValue;

//int divide_row = 1;
//int m_width = 640;
//int m_height = 480;
//m_width 
//m_height
//double pixel_unit_in800mm = 1.606;
//double pixel_unit;
int max_depth_data = 3600;
int min_depth_data = 1000;
int image_count = 0;


int left_pixel_leg;
//int right_pixel_leg;
int mid_pixel_leg;
int goods_border_column;
int goods_border_row;
int pallet_border_row;  
double oversize_length;
//std::stringstream ss;


using namespace std;
using namespace cv;
using namespace openni;



void SampleViewer::TestFunction()
{
	std::cout << "this is test_function." << std::endl;
}
int SampleViewer::CalculateOverSizePixel(const ushort &goods_depth_data ,const ushort &pallet_depth_data)
{
	std::cout << "goods_depth_data is "  << goods_depth_data << std::endl;
	std::cout << "pallet_depth_data is "  << pallet_depth_data << std::endl;

	double depth_increment = static_cast<double>(goods_depth_data-pallet_depth_data)*0.33333;

	std::cout << "depth_increment is "  << depth_increment << std::endl;
	int   pixel_increment = static_cast<int>(0.205*depth_increment-3.509);
    std::cout << "pixel_increment is "  << pixel_increment << std::endl;
	int total_pixel = left_pixel_leg - goods_border_column + pixel_increment ;
	std::cout << "total_pixel is "  << total_pixel << std::endl;

	//double overside_length = pixel_unit*total_pixel;
	//return overside_length;
	return total_pixel;

	
}
double SampleViewer::CalculatePixelSize(const ushort &pallet_depth_data) 
{	
	double depth_data_in_mm = static_cast<double>(pallet_depth_data)*0.33333;
	double pixel_unit = 0.002*depth_data_in_mm + 0.017;
	return pixel_unit;

}
double SampleViewer::CalculateOverSizeLength(double pixel_unit , int pixel_count)
{
	if(pixel_count <= 0)
	{
		return 0;
	}
	else
	{
		
		return pixel_unit*pixel_count;
	}
}

void SampleViewer::CalculatePalletBorder()
{
	//left_pixel_leg = 1;
	//right_pixel_leg = 1;
	int tem_index = 0;
	const ushort *ptr;
	
	for (int j = 500; j < 530; j++)
	{
		ptr = img.ptr<ushort>(j);
		for(int i = 0;i < 285; i++)
		{	
			//if( ptr[i]-ptr[i+7] > 200 && left_pixel_leg == 1 && ptr[i]> min_depth_data &&  ptr[i+7]> min_depth_data)
			if( ptr[i]-ptr[i+7] > 200  && ptr[i]> min_depth_data &&  ptr[i+7]> min_depth_data)
			{
				left_pixel_leg = i+5;
				pallet_border_row = j;
				tem_index = 1;
				//continue;
				break;
			}

		}

		//if(left_pixel_leg !=1)
		if (tem_index == 1)
			break;
	}
	std::cout << "pallet_border_row is " << pallet_border_row << std::endl;
	//std::fstream outfile;
	//outfile.open("data.txt",ios::out|ios::app);
	//outfile<<"卡板边界行数为: " << pallet_border_row  << " ,卡板边界列数为:  " << left_pixel_leg <<std::endl;
	//for (int i = 0; i < 350; i++)
	//{
	//	const ushort *ptr = img.ptr<ushort>(pallet_border_row);
	//	cout << "ptr_" << pallet_border_row << "_" << i << " is "<< ptr[i] << endl;
	//	//outfile << "ptr[ " << pallet_border_row << "]: " << ptr[i] <<std::endl;  
	//}

	//time_t now = time(0);
	//tm *ltm = localtime(&now);
	//cout << "时间: "<< ltm->tm_hour << ":";
   	//	cout << ltm->tm_min << ":";
   	//	cout << ltm->tm_sec << endl;
	//outfile << "时间: "<< ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec << endl;
   						
	//outfile << "******************************" << std::endl;
	//outfile.close();

	std::cout << "left_pixel_leg is " << left_pixel_leg << "\n";
	mid_pixel_leg = left_pixel_leg+20 ;
	std::cout << "mid_pixel_leg is " << mid_pixel_leg<< std::endl;
	
	//for plastic pallet only
	//double pixel_unit = CalculatePixelSize(ptr[mid_pixel_leg]);
	//double plastic_leftleg_pixel_increment = static_cast<int>(40.0/pixel_unit);
	//left_pixel_leg= left_pixel_leg-plastic_leftleg_pixel_increment;
	
	
}


void SampleViewer::CalculateGoodsBorder()
{	
	// for plastic pallet

	//int temp_count=0;
	std::vector<int> v_pixel_count;
	for (int  i = 0; i < m_height; i++)
	{	
		int pixel_count =0;
		
		for (int j = 0; j < m_width; j++)
		{
			const uchar *ptr = dstImage.ptr<uchar>(j);
			if(ptr[i] > 0)
			{
				pixel_count++;
			}
		}
		v_pixel_count.push_back(pixel_count);
	}

	
	for (int i = 0; i < 450; i++)
	{	
		//std::cout << "v_pixel_count_"<<i <<" " << v_pixel_count[i]  << std::endl;
		if (v_pixel_count[i+4] - v_pixel_count[i] > 100)
		{
			goods_border_column = i+2;
		}

		
	}

	
		//if(pixel_count - temp_count > 40)
		//{
		//	goods_border_column = i;
		//	break;
		//} 
		//temp_count = pixel_count;
		//std::cout << "pixel_count " <<i <<" is " << pixel_count << std::endl;
		
	
	//goods_border_row = 1;
	//goods_border_column = 1;
	
	std::cout << "goods_border_column is " << goods_border_column << std::endl; 

	for (int i = 0; i < m_width; i++)
	{	
		const uchar *ptr = dstImage.ptr<uchar>(i);
		if(ptr[goods_border_column] > 0)
		{
			goods_border_row = i;
			break;
		}
			
	}

	std::cout << "goods_border_row is "<< goods_border_row << std::endl;
	
}

void SampleViewer::Histogram()
{
	cv::Mat histo_pallet_515row(img.size(), CV_8UC1, cv::Scalar(0));
	const ushort *ptr_515 = img.ptr<ushort>(515);
	
	cv::Mat histo_goods_350row(img.size(), CV_8UC1, cv::Scalar(0));
	const ushort *ptr_350 = img.ptr<ushort>(350);
	for(int i = 0;i < 350; i++)
	{

		line(histo_pallet_515row, cv::Point(i, m_width), cv::Point(i, m_width - (double)(ptr_515[i]*m_width)/maxValue), cv::Scalar(255),1,8);
		line(histo_goods_350row, cv::Point(i, m_width), cv::Point(i, m_width - (double)(ptr_350[i]*m_width)/maxValue), cv::Scalar(255),1,8);
		
	}
	
	cv::Mat histo_pallet_goods(img.size(), CV_8UC1, cv::Scalar(0));
	for(int i = 200 ; i <= 510 ;i++ )
	{
		const ushort *ptr = img.ptr<ushort>(i);	
		line(histo_pallet_goods, cv::Point(i-100, m_width), cv::Point(i-100, m_width - (double)(ptr[mid_pixel_leg]*m_width)/maxValue), cv::Scalar(255),1,8);
			
	}

	imshow("histo_pallet_goods",histo_pallet_goods);
	imshow("histo_pallet_515row",histo_pallet_515row);
	imshow("histo_goods_350row",histo_goods_350row);

}


void SampleViewer::EliminateBackGround( cv::Mat& inputGray )
{
	for (int y = 0; y < inputGray.rows; y++)
		for (int x = 0; x < inputGray.cols; x ++)
		{	
			
			if (inputGray.at<ushort>(y, x) > max_depth_data)
			{
				inputGray.at<ushort>(y, x) = 0 ;
			}

			if (y >= 600)
			{
				inputGray.at<ushort>(y, x) = 0;
			}
		}	
	
	
}

void SampleViewer::LargestConnecttedComponent(cv::Mat srcImage, cv::Mat &dstImage)
{
    cv::Mat temp;
    cv::Mat labels;
    srcImage.copyTo(temp);

    //1. 标记连通域
    int n_comps = connectedComponents(temp, labels, 4, CV_16U);
    std::vector<int> histogram_of_labels;
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

SampleViewer::SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color):
	m_device(device), m_depthStream(depth), m_colorStream(color), m_streams(NULL)
{	
}

SampleViewer::~SampleViewer()
{
}

openni::Status SampleViewer::init()
{
	openni::VideoMode depthVideoMode;
	openni::VideoMode colorVideoMode;
	if (m_depthStream.isValid()) {
		depthVideoMode = m_depthStream.getVideoMode();
		colorVideoMode = m_colorStream.getVideoMode();
		m_width = depthVideoMode.getResolutionX();
		m_height = depthVideoMode.getResolutionY();
		std::cout << "m_height is " << m_height << std::endl;
		std::cout << "m_width is " << m_width << std::endl;
		std::cout << "depthVideoMode.getpixelformat is " << depthVideoMode.getPixelFormat() << std::endl;
		std::cout << "openni::OpenNI::getDepthValueUnit_mm is " << openni::OpenNI::getDepthValueUnit_mm << std::endl;
		
		int colorWidth = colorVideoMode.getResolutionX();
		int colorHeight = colorVideoMode.getResolutionY();
		std::cout << "colorWidth is "  << colorWidth  <<"\n" 
			<< "colorHeight is "  << colorHeight  << std::endl;
	}
	else
	{
		printf("Error - expects at least one of the streams to be valid...\n");
		return openni::STATUS_ERROR;
	}

	m_streams = new openni::VideoStream*[2];
	m_streams[0] = &m_depthStream;
	m_streams[1] = &m_colorStream;
	
	return openni::STATUS_OK;

}
void SampleViewer::ImageFilter(cv::Mat &src_image)
{
	for (int i = 480; i < 530 ; i++)
    {
        ushort *ptr = src_image.ptr<ushort>(i);
        for (int j = 0; j < 350; j++)
        {
            //std::cout << i<< "_ptr_pre_"<<j << " is " <<ptr[j] << std::endl;
            if (ptr[j] == 0)
            {
                vector<ushort> tem_vector;
                //std::cout << "ptr_pre_"<<i << " is " <<ptr[i] << std::endl;
                cv::Mat cut_c(src_image, cv::Rect(j+1,i-2,5,5));
                
                for (int k = 0; k < cut_c.rows; k++)
                {
                    const ushort *ptr_cut_c = cut_c.ptr<ushort>(k);
                    for (int l = 0; l < cut_c.cols; l++)
                    {
                        //std::cout <<i<<"_"<<j<< "_ptr_cut_c[l]_" << ptr_cut_c[l]<< std::endl;
                        if (ptr_cut_c[l] != 0)
                        {
                            tem_vector.push_back(ptr_cut_c[l]);
                        }
                       
                    }
                }
                //std::cout << "tem_vector.size() is " << tem_vector.size()<< std::endl;
                if (tem_vector.size() != 0)
                {
                    int sum = accumulate(tem_vector.begin() , tem_vector.end() , 0);
                    ptr[j] = sum/tem_vector.size();
                }
                else
                {
                    continue;
                }
                
                //std::cout << "sum is "<< sum << std::endl;
                //std::cout << "tem_vector.size() is "<< tem_vector.size() << std::endl;
                
                
                //std::cout << "ptr_"<<i << " is " <<ptr[i] << std::endl;
            }
            //std::cout << i<< "_ptr_"<<j << " is " <<ptr[j] << std::endl;
        }
    }
}

double SampleViewer::display()
{
	
	//calculate overside 
		
	    int changedIndex;
	    openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
	    if (rc != openni::STATUS_OK)
	    {
		    printf("Wait failed\n");
		    return 1;
	    }
		std::cout << "changedIndex is "  << changedIndex << std::endl;
	    switch (changedIndex)
	    {
	    case 0:
			m_depthStream.readFrame(&m_depthFrame); break;
	    case 1:
	        m_colorStream.readFrame(&m_colorFrame); break;
	    default:
		    printf("Error in wait\n");
	    }
		
		/*
		if (m_colorFrame.isValid())
		{
			cv::Mat colorMat = cv::Mat(m_colorFrame.getHeight(), m_colorFrame.getWidth(), CV_8UC3, (void*)m_colorFrame.getData());
			cv::Mat colorBGR;
			cv::cvtColor(colorMat, colorBGR, CV_RGB2BGR);
			cv::imshow("color.jpg", colorBGR);
			
			
		}
		*/
		if (m_depthFrame.isValid())
		{
			cv::Mat depthMat = cv::Mat(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1, (void*)m_depthFrame.getData());
			cv::Mat temp ,image_normal,converted_image;
			

			transpose(depthMat, temp);
			flip(temp,img,1);
		
			cv::Point minLoc, maxLoc;
    		minMaxLoc(img, &minValue, &maxValue, &minLoc, &maxLoc);
			EliminateBackGround(img);
			ImageFilter(img);

			img.convertTo(converted_image, CV_8UC1);
			LargestConnecttedComponent(converted_image, dstImage);
			std::cout << "dstImage.depth() "  << dstImage.depth()<< std::endl;
			std::cout << "dstImage.type() " <<dstImage.type() << std::endl;
			imshow("dstImage",dstImage);
			
			CalculatePalletBorder();
			CalculateGoodsBorder();
			Histogram();

			if (left_pixel_leg  > goods_border_column)
			{
				const ushort *ptr_1 = img.ptr<ushort>(goods_border_row);
				const ushort *ptr_2= img.ptr<ushort>(pallet_border_row);
				//std::cout << "ptr[goods_border_column+15] is  " << ptr[goods_border_column+15] << std::endl;

				double pixel_unit = CalculatePixelSize(ptr_2[mid_pixel_leg]);
				std::cout << "pixel_unit is " << pixel_unit <<"mm/pixel " << std::endl;
				int oversize_pixel = CalculateOverSizePixel(ptr_1[goods_border_column],ptr_2[mid_pixel_leg]);
				oversize_length = CalculateOverSizeLength(pixel_unit, oversize_pixel);

			}
			else
			{
				oversize_length = 0;
			}

			std::fstream outfile;
			outfile.open("data.txt",ios::out|ios::app);
			struct timeval tv;
			tv.tv_sec = std::time(0);
			//std::tm* now = std::localtime(&t);
			gettimeofday(&tv, NULL);
			std::tm* now = std::localtime(&tv.tv_sec);
			int hour = now->tm_hour;
			int min  = now->tm_min;
			int sec  = now->tm_sec;
			int year = now->tm_year + 1900;
			int month = now->tm_mon + 1;
			int day = now->tm_mday;

			//to_string(now->tm_year + 1900) + "-" +
			//to_string(now->tm_mon + 1)     + "-" +
			//to_string(now->tm_mday)        + " " +
			//tv.tv_usec/1000
			outfile << year<< "-" << month<< "-" << day 
					<<  " " << hour << ":" << min<< ":" << sec <<"." << tv.tv_usec/1000 << " "<< oversize_length << " mm" << endl;
			//outfile << "******************************" << std::endl;
			outfile.close();


			
			std::cout << "oversize_length is  " << oversize_length << " mm" << std::endl; 
			std::cout << "***********************" << std::endl;

			normalize(img, image_normal, 0, 255, cv::NORM_MINMAX,CV_8UC1);
		
			line(image_normal, cv::Point(left_pixel_leg, 0), cv::Point(left_pixel_leg, m_width), cv::Scalar(255), 1, 8);
			line(image_normal, cv::Point(goods_border_column, 0), cv::Point(goods_border_column, m_width), cv::Scalar(255), 1, 8);

			stringstream s;
			string temp_pos;
			s << oversize_length;
			s >> temp_pos;
			cv::putText(image_normal, "oversize_length "+temp_pos+" mm", cv::Point(0,30), 2, 1, cv::Scalar(255), 0.1, 8, 0);
			cv::putText(image_normal, "palletborder", cv::Point(left_pixel_leg,70), 2, 1, cv::Scalar(255), 0.1, 8, 0);
			cv::putText(image_normal, "goodsborder", cv::Point(goods_border_column,110), 2, 1, cv::Scalar(255), 0.1, 8, 0);
			//cv::putText(image_normal, "leg_right", cv::Point(right_pixel_leg,50), 2, 0.5, cv::Scalar(255), 0.1, 8, 0);

			imshow("image_normal",image_normal);
		
			
		return oversize_length;
		}

}


