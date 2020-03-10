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
#ifndef _ONI_SAMPLE_VIEWER_H_
#define _ONI_SAMPLE_VIEWER_H_

#include <h_test/OpenNI.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>

#define MAX_DEPTH 4096
#define COLOR_GRAD 96
#define GAMMA_TABLE_SIZE 256
#define GAMMA_NORMALIZE_MAXFLOAT 255.0f
#define GAMMA_NORMALIZE_MAX 255
#define GAMMA_FEATURE 2.2f

enum DisplayModes
{
	DISPLAY_MODE_OVERLAY,
	DISPLAY_MODE_DEPTH,
	DISPLAY_MODE_IMAGE
};

class SampleViewer
{
public:
	SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color);
	SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth);
	virtual ~SampleViewer();

	virtual openni::Status init(int argc, char **argv);
	virtual openni::Status run();	//Does not return
	void Histogram();
	void CalculatePalletBorder();
	void CalculateGoodsBorder();
	double CalculatePixelSize(const ushort &pallet_depth_data);
	int CalculateOverSizePixel(const ushort &goods_depth_data ,const ushort &pallet_depth_data); 
	double CalculateOverSizeLength(double pixel_unit , int pixel_count);
	void EliminateBackGround( cv::Mat& inputGray );
	void LargestConnecttedComponent(cv::Mat srcImage, cv::Mat &dstImage);
	void TestFunction();

protected:
	virtual void display();
	virtual void displayPostDraw(){};	// Overload to draw over the screen image

	virtual void onKey(unsigned char key, int x, int y);

	virtual openni::Status initOpenGL(int argc, char **argv);
	void initOpenGLHooks();

	openni::VideoFrameRef		m_depthFrame;
	// openni::VideoFrameRef		m_colorFrame;

	openni::Device&			m_device;
	openni::VideoStream&			m_depthStream;
	// openni::VideoStream&			m_colorStream;
	openni::VideoStream**		m_streams;

private:
	SampleViewer(const SampleViewer&);
	SampleViewer& operator=(SampleViewer&);

	static SampleViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);

	float			m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;
	DisplayModes		m_eViewState;
	openni::RGB888Pixel*	m_pTexMap;
	int			m_width;
	int			m_height;
	unsigned char m_ColorR[COLOR_GRAD];
	unsigned char m_ColorG[COLOR_GRAD];
	unsigned char m_ColorB[COLOR_GRAD];
	unsigned char m_GammaTable[GAMMA_TABLE_SIZE];
};


#endif // _ONI_SAMPLE_VIEWER_H_
