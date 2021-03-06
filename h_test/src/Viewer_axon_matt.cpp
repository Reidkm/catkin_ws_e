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
#ifndef _CRT_SECURE_NO_DEPRECATE
	#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "h_test/Viewer_axon.h"
#include "math.h"
#include <cstdio>
#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <istream>
#include <ctime>
#include <sstream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
/*
#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif
*/
#include "h_test/OniSampleUtilities.h"

#define GL_WIN_SIZE_X	1280
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

cv::Mat temp,img,image_normal,converted_image,dstImage;
double minValue, maxValue;
cv::Point minLoc, maxLoc;

int img_height = 640;
int img_width = 480;
int max_depth_data = 3600;
int min_depth_data = 1000;
int image_count = 0;

int left_pixel_leg;
int right_pixel_leg;
int mid_pixel_leg;
int min_col;
int row_min_cols;


using namespace std;

SampleViewer* SampleViewer::ms_self = NULL;


int CalculateOverSidePixel(const ushort &goods_depth_data ,const ushort &pallet_depth_data)
{
	std::cout << "goods_depth_data is "  << goods_depth_data << std::endl;
	std::cout << "pallet_depth_data is "  << pallet_depth_data << std::endl;

	double depth_increment = static_cast<double>(goods_depth_data-pallet_depth_data)*0.33333;

	std::cout << "depth_increment is "  << depth_increment << std::endl;
	int   pixel_increment = static_cast<int>(0.205*depth_increment-3.509);
    std::cout << "pixel_increment is "  << pixel_increment << std::endl;
	int total_pixel = left_pixel_leg - min_col + pixel_increment ;
	std::cout << "total_pixel is "  << total_pixel << std::endl;

	
	return total_pixel;

	
}
double CalculatePixelSize(const ushort &pallet_depth_data) 
{	
	double depth_data_in_mm = static_cast<double>(pallet_depth_data)*0.33333;
	double pixel_unit = 0.002*depth_data_in_mm + 0.017;
	return pixel_unit;

}
double CalculateOverSideLength(double pixel_unit , int pixel_count)
{
	if(pixel_count <= 0)
	{
		return 0;
	}
	else
	{
		double overside_length = pixel_unit*pixel_count;
		return overside_length;
	}
}

void CalculatePalletBorder()
{
	left_pixel_leg = 1;
	right_pixel_leg = 1;
	const ushort *ptr = img.ptr<ushort>(515);
	for(int i = 0;i < 285; i++)
	{
		if( ptr[i]-ptr[i+10] > 500 && left_pixel_leg == 1 && ptr[i]> min_depth_data &&  ptr[i+10]> min_depth_data)
		{
			left_pixel_leg = i+10;
			continue;
		}
		if(ptr[i+20]-ptr[i] > 500 && right_pixel_leg == 1 && ptr[i]> min_depth_data && ptr[i+20]> min_depth_data)
		{
				
			right_pixel_leg = i+10 ;
            break;
		}

	}
	std::cout << "left_pixel_leg is " << left_pixel_leg << "\n"
		 << "right_pixel_leg is " << right_pixel_leg << std::endl;
		 
	mid_pixel_leg =  (left_pixel_leg+right_pixel_leg)/2 ;
	std::cout << "mid_pixel_leg is " << mid_pixel_leg << std::endl;

}
void CalculateGoodsBorder()
{	
	min_col = 300;
	
	for(int i = 0; i < img_height; i++)
	{
		const ushort *ptr = img.ptr<ushort>(i);

		for(int j = 0;j <mid_pixel_leg ;j++ )
		{
			int check_border_count = 0 ;
			if((ptr[j]-ptr[j+2])> 500 && ptr[j] > 2000 && ptr[j+2] > 2000 && ptr[j] < max_depth_data && ptr[j+2] < max_depth_data   )
			{
					
				for(int  i = 0; i < 100; i++)
				{	
					if(ptr[j+i] > min_depth_data && ptr[j+i] < max_depth_data )
					{
						check_border_count= check_border_count + 1;
					}
				}
				if(check_border_count > 97 && j+4 < min_col)
				{
					
					min_col = j+4 ;
					row_min_cols= i;
					break;
				}
				else
				{
					continue;
				}
				
			}
		}
	}
	std::cout << "row_min_cols is "<< row_min_cols << std::endl;
	std::cout << "min_col is " << min_col << std::endl; 
	

}
void Histogram()
{
	cv::Mat histo_pallet_515row(img.size(), CV_8UC1, cv::Scalar(0));
	const ushort *ptr_515 = img.ptr<ushort>(515);
	
	cv::Mat histo_goods_350row(img.size(), CV_8UC1, cv::Scalar(0));
	const ushort *ptr_350 = img.ptr<ushort>(350);
	for(int i = 0;i < 285; i++)
	{
		
		line(histo_pallet_515row, cv::Point(i, img_height), cv::Point(i, img_height - (double)(ptr_515[i]*img_height)/maxValue), cv::Scalar(255),1,8);
		
		line(histo_goods_350row, cv::Point(i, img_height), cv::Point(i, img_height - (double)(ptr_350[i]*img_height)/maxValue), cv::Scalar(255),1,8);

	}

	

	cv::Mat histo_pallet_goods(img.size(), CV_8UC1, cv::Scalar(0));
	for(int i = 200 ; i <= 510 ;i++ )
	{
		const ushort *ptr = img.ptr<ushort>(i);
			
		
		line(histo_pallet_goods, cv::Point(i-100, img_height), cv::Point(i-100, img_height - (double)(ptr[mid_pixel_leg]*img_height)/maxValue), cv::Scalar(255),1,8);
			
	}
	
		
	imshow("histo_pallet_goods",histo_pallet_goods);
	imshow("histo_pallet_515row",histo_pallet_515row);
		
	imshow("histo_goods_350row",histo_goods_350row);

}
/* Matt
void SampleViewer::glutIdle()
{
	glutPostRedisplay();
}
void SampleViewer::glutDisplay()
{
	SampleViewer::ms_self->display();
}
void SampleViewer::glutKeyboard(unsigned char key, int x, int y)
{
	SampleViewer::ms_self->onKey(key, x, y);
}
*/
void extremeValue(const cv::Mat& inputGray,int& pixel_max ,int& pixel_min)
{
    
	unsigned short grayValue; 
	unsigned short maxValue = 1;
	unsigned short minValue = 4000;
	for (int y = 0; y < inputGray.rows; y++)
		for (int x = 0; x < inputGray.cols; x ++)
		{
           
			grayValue = inputGray.at<ushort>(y, x);
			maxValue = std::max(maxValue, grayValue);
			pixel_max = (int)maxValue ;
			
			if(grayValue > 100)
			{
				minValue = std::min(minValue, grayValue);
				pixel_min = (int)minValue;
				

			}
		}
        
}

void Eliminatebackground( cv::Mat& inputGray )
{
	for (int y = 0; y < inputGray.rows; y++)
		for (int x = 0; x < inputGray.cols; x ++)
		{	
			
			if (inputGray.at<ushort>(y, x) > max_depth_data)
			{
				inputGray.at<ushort>(y, x) = 0 ;
			}
			if (y >= 550)
			{
				inputGray.at<ushort>(y, x) = 0;
			}
		}	
	
	
}

int minColumnGoods(const cv::Mat& inputGray)
{
	int mincolnum = 200;
	int colnum = 2 ;
	for (int y = 50; y < 421; y++)
	{
		for (int x = 0; x < inputGray.cols; x ++)
		{	
			
			if(int(inputGray.at<ushort>(y, x)) >2000)
			{
				
				colnum = x;
				mincolnum = std::min(colnum,mincolnum);
				break;
			}
		}	
	}
	return mincolnum;
}

int colPalletPixel( const cv::Mat& inputGray,int& firstcol)
{
	
	int secondcol;
	int k = 0;
	
	for (int x = 0; x < inputGray.cols; x ++)
	{	
			
		if(k==0)
		{
			if(int(inputGray.at<ushort>(471, x)) >0)
			{
				firstcol = x;
				
				k = 1;
				continue;
			}
		}
		else
		{
			if(int(inputGray.at<ushort>(471, x)) == 0)
			{
				secondcol = x;
				
				break;
			}
		}
		
	}	
	int num_palletlegpixel = secondcol - firstcol;
	
	return num_palletlegpixel;
}

cv::Mat scaleGray(const cv::Mat& inputGray)
{
    cv::Mat outputGray(inputGray.size(), CV_8UC3);
    unsigned char grayValue, maxValue = 1;
    for (int y = 0; y < inputGray.rows; y++)
        for (int x = 0; x < inputGray.cols; x ++)
        {
            grayValue = inputGray.at<uchar>(y, x);
            maxValue = std::max(maxValue, grayValue);
        }
         
    float scale = 255.0 / maxValue;   
    for (int y = 0; y < inputGray.rows; y++)
        for (int x = 0; x < inputGray.cols; x ++)
        {
            outputGray.at<uchar>(y, x) = static_cast<unsigned char>(inputGray.at<uchar>(y, x) * scale + 0.5);
        }
 
    return outputGray;
}

cv::Mat gray2pseudocolor(const cv::Mat& scaledGray)
{
    cv::Mat outputPseudocolor(scaledGray.size(), CV_8UC3);
    unsigned char grayValue;
    for (int y = 0; y < scaledGray.rows; y++)
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<uchar>(y, x);
            cv::Vec3b& pixel = outputPseudocolor.at<cv::Vec3b>(y, x);
            pixel[0] = abs(255 - grayValue);
            pixel[1] = abs(127 - grayValue);
            pixel[2] = abs(0 - grayValue);
        }
 
    return outputPseudocolor;
}

void LargestConnecttedComponent(cv::Mat srcImage, cv::Mat &dstImage)
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





// SampleViewer::SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color) :
// 	m_device(device), m_depthStream(depth), m_colorStream(color), m_streams(NULL), m_eViewState(DEFAULT_DISPLAY_MODE), m_pTexMap(NULL)

// {
// 	ms_self = this;
// 	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
// }

SampleViewer::SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth) :
	m_device(device), m_depthStream(depth), m_streams(NULL), m_eViewState(DEFAULT_DISPLAY_MODE), m_pTexMap(NULL)

{
	ms_self = this;
	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
	for (int i = 0; i < COLOR_GRAD; i++)
	{
		if (i < COLOR_GRAD / 3)
		{
			m_ColorR[i] = 255;
			m_ColorG[i] = i*8;
			m_ColorB[i] = 0;
		}
		else if (i < COLOR_GRAD * 2 / 3)
		{
			m_ColorR[i] = 255-(i- COLOR_GRAD / 3)*8;
			m_ColorG[i] = 255;
			m_ColorB[i] = (i - COLOR_GRAD / 3) * 8;
		}
		else
		{
			m_ColorR[i] = 0;
			m_ColorG[i] = 255 - (i - COLOR_GRAD*2 / 3) * 8;
			m_ColorB[i] = 255;
		}

	}

	for (int i = 0; i < GAMMA_TABLE_SIZE; i++)
	{
		m_GammaTable[i] = (unsigned char)(GAMMA_NORMALIZE_MAXFLOAT * pow(i / GAMMA_NORMALIZE_MAXFLOAT, 1 / GAMMA_FEATURE));
	}
	
}

SampleViewer::~SampleViewer()
{
	delete[] m_pTexMap;

	ms_self = NULL;

	if (m_streams != NULL)
	{
		delete []m_streams;
	}
}

openni::Status SampleViewer::init(int argc, char **argv)
{
	openni::VideoMode depthVideoMode;
	// openni::VideoMode colorVideoMode;

/* 	if (m_depthStream.isValid() && m_colorStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
		colorVideoMode = m_colorStream.getVideoMode();

		int depthWidth = depthVideoMode.getResolutionX();
		int depthHeight = depthVideoMode.getResolutionY();
		int colorWidth = colorVideoMode.getResolutionX();
		int colorHeight = colorVideoMode.getResolutionY();

		if (depthWidth == colorWidth &&
			depthHeight == colorHeight)
		{
			m_width = depthWidth;
			m_height = depthHeight;
		}
		else
		{
			printf("Error - expect color and depth to be in same resolution: D: %dx%d, C: %dx%d\n",
				depthWidth, depthHeight,
				colorWidth, colorHeight);
			return openni::STATUS_ERROR;
		}
	}
	else if (m_depthStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
		m_width = depthVideoMode.getResolutionX();
		m_height = depthVideoMode.getResolutionY();
	}
	else if (m_colorStream.isValid())
	{
		colorVideoMode = m_colorStream.getVideoMode();
		m_width = colorVideoMode.getResolutionX();
		m_height = colorVideoMode.getResolutionY();
	}
	else
	{
		printf("Error - expects at least one of the streams to be valid...\n");
		return openni::STATUS_ERROR;
	} */

	if (m_depthStream.isValid()) {
		depthVideoMode = m_depthStream.getVideoMode();
		m_width = depthVideoMode.getResolutionX();
		m_height = depthVideoMode.getResolutionY();
		std::cout << "height is " << m_height << std::endl;
		std::cout << "m_width is " << m_width << std::endl;
		std::cout << "depthVideoMode.getpixelformat is " << depthVideoMode.getPixelFormat() << std::endl;
		std::cout << "openni::OpenNI::getDepthValueUnit_mm is " << openni::OpenNI::getDepthValueUnit_mm << std::endl;
		//const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
		int middleIndex = (m_height+1)*m_width/2;
		std::cout << "middleIndex is " << middleIndex << std::endl;
		//std::cout << "middlepixel depth is  " << pDepthRow[middleIndex] << std::endl;
	}
	else
	{
		printf("Error - expects at least one of the streams to be valid...\n");
		return openni::STATUS_ERROR;
	}

	m_streams = new openni::VideoStream*[1];
	m_streams[0] = &m_depthStream;
	// m_streams[1] = &m_colorStream;

	// Texture map init
	m_nTexMapX = MIN_CHUNKS_SIZE(m_width, TEXTURE_SIZE);
	m_nTexMapY = MIN_CHUNKS_SIZE(m_height, TEXTURE_SIZE);
	m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];

	//return initOpenGL(argc, argv);
	return openni::STATUS_OK;

}
openni::Status SampleViewer::run()	//Does not return
{
	//glutMainLoop();
	while(1)
	{
		SampleViewer::ms_self->display();
	}

	return openni::STATUS_OK;
}
void SampleViewer::display()
{
	//std::cout << "1" << std::endl;	
	int changedIndex;
	openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 1, &changedIndex);
	if (rc != openni::STATUS_OK)
	{
		printf("Wait failed\n");
		return;
	}

	switch (changedIndex)
	{
	case 0:
		m_depthStream.readFrame(&m_depthFrame); break;
	// case 1:
	// 	m_colorStream.readFrame(&m_colorFrame); break;
	default:
		printf("Error in wait\n");
	}
/* Matt
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);
*/
	if (m_depthFrame.isValid())
	{
		calculateHistogram(m_pDepthHist, MAX_DEPTH, m_depthFrame);
	}

	memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

	// check if we need to draw depth frame to texture
	if ((m_eViewState == DISPLAY_MODE_OVERLAY ||
		m_eViewState == DISPLAY_MODE_DEPTH) && m_depthFrame.isValid())
	{
		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
		openni::RGB888Pixel* pTexRow = m_pTexMap + m_depthFrame.getCropOriginY() * m_nTexMapX;
		int rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
		//std::cout << "middlepixel depth is  " << pDepthRow[153920] << std::endl;
		for (int y = 0; y < m_depthFrame.getHeight(); ++y)
		{
			const openni::DepthPixel* pDepth = pDepthRow;
			openni::RGB888Pixel* pTex = pTexRow + m_depthFrame.getCropOriginX();

			for (int x = 0; x < m_depthFrame.getWidth(); ++x, ++pDepth, ++pTex)
			{
				if (*pDepth != 0)
				{
					//int nHistValue = m_pDepthHist[*pDepth];
					int nColorIndex = (*pDepth)*COLOR_GRAD / MAX_DEPTH;
					pTex->r = m_ColorR[nColorIndex];
					pTex->g = m_ColorG[nColorIndex];
					pTex->b = m_ColorB[nColorIndex];
				}
			}

			pDepthRow += rowSize;
			pTexRow += m_nTexMapX;
		}
	}
/* Matt
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);

	glBegin(GL_QUADS);

	int nXRes = m_width;
	int nYRes = m_height;

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float)nXRes/(float)m_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float)nXRes/(float)m_nTexMapX, (float)nYRes/(float)m_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float)nYRes/(float)m_nTexMapY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();

	// Swap the OpenGL display buffers
	glutSwapBuffers();

*/
	//calculate overside 

	if(m_depthFrame.isValid())
	{
		
		cv::Mat src = cv::Mat(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1, (void*)m_depthFrame.getData());
		transpose(src, temp);
		flip(temp,img,1);
		
    	minMaxLoc(img, &minValue, &maxValue, &minLoc, &maxLoc);
		CalculatePalletBorder();
		CalculateGoodsBorder();
		Histogram();
		const ushort *ptr_1 = img.ptr<ushort>(row_min_cols);
		const ushort *ptr_2= img.ptr<ushort>(515);
		double pixel_unit = CalculatePixelSize(ptr_2[mid_pixel_leg]);
		std::cout << "pixel_unit is " << pixel_unit <<"mm/pixel " << std::endl;
		int overside_pixel = CalculateOverSidePixel(ptr_1[min_col+10],ptr_2[mid_pixel_leg]);
		double overside_length = CalculateOverSideLength(pixel_unit, overside_pixel);
		std::cout << "overside_length is  " << overside_length << " mm" << std::endl; 
		std::cout << "***********************" << std::endl;


		normalize(img, image_normal, 0, 255, cv::NORM_MINMAX,CV_8UC1);
		line(image_normal, cv::Point(0, 514), cv::Point(img_width, 514), cv::Scalar(0), 1, 8);
		line(image_normal, cv::Point(284, 0), cv::Point(284, img_height), cv::Scalar(0), 1, 8);
		imshow("image_normal",image_normal);
		Eliminatebackground(img);
		img.convertTo(converted_image, CV_8UC1);
		LargestConnecttedComponent(converted_image, dstImage);
		imshow("dstImage",dstImage);
		cv::waitKey(1);
	}

}

void SampleViewer::onKey(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		printf("try to close depth stream\n");
		m_depthStream.stop();
		printf("stop Depth OK\n");
		// m_colorStream.stop();
		m_depthStream.destroy();
		printf("Destroy Depth OK\n");
		// m_colorStream.destroy();
		m_device.close();
		printf("Close Device OK\n");
		openni::OpenNI::shutdown();
		printf("Shutdown\n");

		exit (1);
	case '1':
		m_eViewState = DISPLAY_MODE_OVERLAY;
		m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		break;
	case '2':
		m_eViewState = DISPLAY_MODE_DEPTH;
		m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
		break;
	case '3':
		m_eViewState = DISPLAY_MODE_IMAGE;
		m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
		break;
	case 'm':
		m_depthStream.setMirroringEnabled(!m_depthStream.getMirroringEnabled());
		// m_colorStream.setMirroringEnabled(!m_colorStream.getMirroringEnabled());
		break;
	case 'w':
		
		cv::Mat depthMat = cv::Mat(m_depthFrame.getHeight(), m_depthFrame.getWidth(), CV_16UC1, (void*)m_depthFrame.getData());
		//cv::Mat scaledGray = scaleGray(depthMat);
		//cv::Mat pseudocolor = gray2pseudocolor(scaledGray);
		
		//cv::imwrite("pseudocolor.png", pseudocolor);
		cv::imwrite("kkkkkkkkkk.png", depthMat);
		//cv::imwrite("scaledGray.png", scaledGray);
		//std::cout << "img depth is " << depthMat.depth() << std::endl;
		//std::cout << "img type is " << depthMat.type() << std::endl;	
		break;
	}

}
/* edited by Matt

openni::Status SampleViewer::initOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (m_strSampleName);
	// 	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	initOpenGLHooks();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	return openni::STATUS_OK;

}
void SampleViewer::initOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);//glutDisplayFunc,当GLUT认为需要重新显示窗口内容时，都将执行这一函数注册的回调函数
	glutIdleFunc(glutIdle);
}
*/
