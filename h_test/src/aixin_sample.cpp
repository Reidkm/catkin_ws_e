/**************************************************************
 *                                                            *
 *															  *
 *                                                            *
 *                                                            *
 *************************************************************/
 
#include <h_test/OpenNI.h>
#include "h_test/AXonLink.h"
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
using namespace openni;
typedef struct XYZRGB
{
	float x;
	float y;
	float z;
	uint8_t r;
	uint8_t g;
	uint8_t b;
}XYZRGB;
enum REGISTRATION_TYPE
{
	setImageRegistrationMode_OFF_API = 1, //对齐接口未打开时，使用openni API 来做对齐
	setImageRegistrationMode_OFF_CAMERAPARAM =2,// 对齐接口未打开时，通过相机参数来做对齐；
	setImageRegistrationMode_ON_COLOR2DEPTH =3, //打开对齐接口自动对齐；
};
CamIntrinsicParam* GetColorInstrinsicParamByResolution(int nWidth, int nHeight, AXonLinkCamParam* allParam)
{
	// Find Color Sensor Intrinsic Parameters
	CamIntrinsicParam *pstParam = NULL;
	for(int i = 0; i < AXON_LINK_SUPPORTED_PARAMETERS; i++)
	{
		pstParam = &allParam->astColorParam[i];
		if(pstParam->ResolutionX == nWidth
			&& pstParam->ResolutionY == nHeight)
			return pstParam;
	}

	return NULL;
}
CamIntrinsicParam* GetDepthInstrinsicParamByResolution(int  nWidth, int nHeight,AXonLinkCamParam* allParam )
{
	// Find Color Sensor Intrinsic Parameters
	CamIntrinsicParam *pstParam = NULL;
	for(int i = 0; i < AXON_LINK_SUPPORTED_PARAMETERS; i++)
	{
		pstParam = &allParam->astDepthParam[i];
		if(pstParam->ResolutionX == nWidth
			&& pstParam->ResolutionY == nHeight)
			return pstParam;
	}

	return NULL;
}
void printfIntParam(CamIntrinsicParam* intParam)
{
	if(intParam == NULL)
		return;
	printf("resolution_x %d \nresolution_y %d \n cx = %f\n cy = %f\n fx = %f\n fy = %f\n  k1 = %f\n k2 = %f\n k3 = %f\n k4 = %f\n k5 = %f\n k6 = %f\n",intParam->ResolutionX, intParam->ResolutionY, intParam->cx, intParam->cy, intParam->fx, intParam->fy, intParam->k1, intParam->k2, intParam->k3, intParam->k4, intParam->k5, intParam->k6);
}
void printfExtParam(AXonLinkCamParam* allParam)
{
	if(allParam == NULL)
		return;
	for(int i = 0; i<9;++i)
	{
		printf("R_Param[%d] = %f\n",i,allParam->stExtParam.R_Param[i]);
	}
	for(int i = 0; i<3; ++i)
	{
		printf("T_Param[%d] = %f\n",i,allParam->stExtParam.T_Param[i]);
	}
}
bool waitAllStream(VideoStream** streams, int allCount, int timeout)
{
	int streamCount = allCount;
	while(1)
	{
		if(streamCount == 0)
			return true;
		int readyStreamIndex = 0;
		if(STATUS_OK != OpenNI::waitForAnyStream(streams, streamCount, &readyStreamIndex, timeout))
		{
			printf("Error: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
			return false;
		}

		streams[readyStreamIndex] = NULL;

		streamCount--;
	}
}
void writePointsCloud2File(std::vector<XYZRGB>& v, std::string fileName )
{
	std::ofstream outfile(fileName,std::ios::trunc);
	if(outfile.is_open())
	{
		int Point_id = 0;
		while(Point_id < v.size())
		{
			char buffer[4096];
			memset(buffer, 0, 4096);

			int usedLen = 0;
			int remainLen = sizeof(buffer) - usedLen;

			bool error = false;
			int minbuffer = 500;
			while(Point_id < v.size() && remainLen > minbuffer)
			{				
				const auto & point = v[Point_id];

				//const int printMaxLen = sprintf_s(buffer + usedLen, remainLen, "%.3f %.3f %.3f %d %d %d\n", point.x, point.y, point.z, point.r, point.g, point.b);
				const int printMaxLen = snprintf(buffer + usedLen, remainLen, "%.3f %.3f %.3f %d %d %d\n", point.x, point.y, point.z, point.r, point.g, point.b);
				if(printMaxLen < 0)
				{
					error = true;
					break;
				}

				if(printMaxLen >= remainLen)
				{
					break;	//exceed buffer
				}

				usedLen += printMaxLen;
				remainLen -= printMaxLen;
				Point_id++;
			}

			if(error)
			{
				break;
			}
			else
			{
				outfile.write(buffer, usedLen);
			}
		}
		outfile.close();
	}


}
void registratonColor2Depth( Device* device, VideoStream* color,  VideoStream* depth, VideoStream* ir,REGISTRATION_TYPE type,std::string fileName)
{
	if(!device->isValid())
		return;
	if(!device->getDepthColorSyncEnabled())
		device->setDepthColorSyncEnabled(true);
	VideoMode colorVideoMode = color->getVideoMode();
	colorVideoMode.setResolution(640, 480);
	color->setVideoMode(colorVideoMode);
	VideoMode depthVideoMode = depth->getVideoMode();
	depthVideoMode.setResolution(640, 480);
	depth->setVideoMode(depthVideoMode);
	VideoMode irVideoMode = ir->getVideoMode();
	irVideoMode.setResolution(640, 480);
	ir->setVideoMode(irVideoMode);

	switch( type)
	{
		case setImageRegistrationMode_ON_COLOR2DEPTH:
		{
			std::vector<XYZRGB> pointCloudv;
			ImageRegistrationMode mode = device->getImageRegistrationMode();
			if(mode != IMAGE_REGISTRATION_COLOR_TO_DEPTH)
				device->setImageRegistrationMode(IMAGE_REGISTRATION_COLOR_TO_DEPTH);


			AXonLinkCamParam camParam;
			int dataSize = sizeof(AXonLinkCamParam);
			Status rc = STATUS_OK;
			rc=device->getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS,
				&camParam,&dataSize);
			CamIntrinsicParam* depthIntParam = GetDepthInstrinsicParamByResolution(640,480,&camParam);
			VideoFrameRef colorFrame, depthFrame,irFrame;
			//color->readFrame(&colorFrame);
			//depth->readFrame(&depthFrame);
			int get_frame_after_Index = 30;
			while(get_frame_after_Index--)
			{
				color->readFrame(&colorFrame);
				depth->readFrame(&depthFrame);
				ir->readFrame(&irFrame);
			}
			PixelFormat format = depth->getVideoMode().getPixelFormat();
			float depth_unit = OpenNI::getDepthValueUnit_mm(format);
			
			std::cout << "format is " << format << std::endl;
			std::cout << "depth_unit is " << depth_unit << std::endl;

			const RGB888Pixel* colorImage = (const RGB888Pixel*)colorFrame.getData();
			const DepthPixel* depthImage = (const DepthPixel* )depthFrame.getData();

			int middleIndex = (depthFrame.getHeight()+1)*depthFrame.getWidth()/2;
			std::cout << "middleIndex is " << middleIndex << std::endl;
			printf("[%08llu]%8d\n",(long long)depthFrame.getTimestamp(), depthImage[middleIndex]);

			int colorRowSize = colorFrame.getStrideInBytes()/sizeof(RGB888Pixel);
			int depthRowSize = depthFrame.getStrideInBytes()/sizeof(DepthPixel);
		
			for( int i = 0; i<depthFrame.getHeight(); ++i)
			{
				const RGB888Pixel* colorRowHead = colorImage;
				const DepthPixel* depthRowHead = depthImage;
				for( int j = 0; j<depthFrame.getWidth();++j) 
				{
					
					int depthValue = (int)(*(depthRowHead+j))*depth_unit;
					if(depthValue>0)
					{
						RGB888Pixel colorValue = (RGB888Pixel)(*(colorRowHead + j));
						XYZRGB point;
						//float pz = depthValue*depth_unit;
						float pz = depthValue;
						float px = (i - depthIntParam->cx) * pz / depthIntParam->fx; 
						float py = (depthIntParam->cy - j) * pz / depthIntParam->fy;

						point.x = px;
						point.y = py;
						point.z = -pz;
						point.r = colorValue.r;
						point.g = colorValue.g;
						point.b = colorValue.b;
						pointCloudv.push_back(point);
					}
				}
				colorImage += colorRowSize;
				depthImage += depthRowSize;
				
			}
			writePointsCloud2File(pointCloudv,"setImageRegistrationMode_ON_COLOR2DEPTH.asc");
		}
		break;
		default:break;
	}
}
int main(int argc, char** argv)
{
	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth, color, ir;
	const char* deviceURI = openni::ANY_DEVICE;
	if (argc > 1)
	{
		deviceURI = argv[1];
	}

	rc = openni::OpenNI::initialize();

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("Error: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}
	AXonLinkCamParam camParam;
	int dataSize = sizeof(AXonLinkCamParam);
	rc=device.getProperty(AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS,
		&camParam,&dataSize);
	CamIntrinsicParam* colorIntParam = GetColorInstrinsicParamByResolution(640,480,&camParam);
	CamIntrinsicParam* depthIntParam = GetDepthInstrinsicParamByResolution(640,480,&camParam);
	printfIntParam(colorIntParam);
	printfIntParam(depthIntParam);
	printfExtParam(&camParam);
	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Error: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("Error: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	rc = color.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		VideoMode mode = color.getVideoMode();
		mode.setPixelFormat(PIXEL_FORMAT_RGB888);
		color.setVideoMode(mode);
		rc = color.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Error: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			color.destroy();
		}
	}
	else
	{
		printf("Error: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}
	rc = ir.create(device, openni::SENSOR_IR);
	if (rc == openni::STATUS_OK)
	{
		rc = ir.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Error: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
			ir.destroy();
		}
	}
	else
	{
		printf("Error: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	}
	if((!depth.isValid()) || (!color.isValid())||(!ir.isValid()))
	{
		printf("Error: No valid streams. Exiting\n");
		OpenNI::shutdown();
		return 2;
	}

	device.setDepthColorSyncEnabled(true);
	device.setImageRegistrationMode(IMAGE_REGISTRATION_COLOR_TO_DEPTH);
/*	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 3;
	}
	rc = color.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 3;
	}
	*/
	VideoStream* allStream[3] = {NULL};
	allStream[0] = &depth;
	allStream[1] = &color;
	allStream[2] = &ir;
	allStream[0] = &color;
	while(!waitAllStream(allStream, 1, 200))
	{
		;
	}
	VideoFrameRef colorFrame, depthFrame,irFrame;
	color.readFrame(&colorFrame);
	depth.readFrame(&depthFrame);
	ir.readFrame(&irFrame);
	if (colorFrame.isValid())
	{
		//ASSERT(colorFrame.getHeight() * colorFrame.getWidth() * sizeof(OniRGB888Pixel) == colorFrame._getFrame()->dataSize);
		cv::Mat colorMat = cv::Mat(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void*)colorFrame.getData());
		cv::Mat colorBGR;
		cv::cvtColor(colorMat, colorBGR, CV_RGB2BGR);
		cv::imwrite("color.jpg", colorBGR);
	}
	if (depthFrame.isValid())
	{
			
		std::vector<int> compression_params; //Depth保存到16位无压缩PNG
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(0);
		//cv::Mat depthMat = cv::Mat(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, (void*)depthFrame.getData());
		
		for(;;)
		{	
			depth.readFrame(&depthFrame);
			cv::Mat depthMat = cv::Mat(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, (void*)depthFrame.getData());
			cv::imshow("img",depthMat);
			cv::waitKey(1);
		}
		
		//cv::imwrite("depth.png", depthMat, compression_params);
	}
	if (irFrame.isValid())
	{
		cv::Mat irMat = cv::Mat(depthFrame.getHeight(), depthFrame.getWidth(), CV_8UC1, (void*)irFrame.getData());
		cv::imwrite("ir.bmp", irMat);
	}
	printf("start registratonColor2Depth\n");
	registratonColor2Depth(&device, &color, &depth,&ir ,setImageRegistrationMode_ON_COLOR2DEPTH,"");
	printf("finish\n");
	color.stop();
	depth.stop();
	ir.stop();
	color.destroy();
	depth.destroy();
	ir.destroy();
	device.close();
	OpenNI::shutdown();
	while (1){};
	return 0;

/*	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Error: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("Error: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	 rc = color.create(device, openni::SENSOR_COLOR);
	 if (rc == openni::STATUS_OK)
	 {
	 	rc = color.start();
	 	if (rc != openni::STATUS_OK)
	 	{
	 		printf("Error: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
	 		color.destroy();
	 	}
	 }
	 else
	 {
	 	printf("Error: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	 }

	if((!depth.isValid()) || (!color.isValid()))
	{
		printf("Error: No valid streams. Exiting\n");
		OpenNI::shutdown();
		return 2;
	}
	
	device.setDepthColorSyncEnabled(true);
	device.setImageRegistrationMode(IMAGE_REGISTRATION_COLOR_TO_DEPTH);
	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 3;
	}
	rc = color.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 3;
	}

	rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
	*/
}

