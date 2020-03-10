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

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp> 
#include <iostream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include "h_test/AXonLink.h"
#include "h_test/OpenNI.h"
#include "h_test/test_m5_openni2_class.h"


using namespace std;
using namespace cv;
using namespace openni;

double length;
int main(int argc, char** argv)
{
	openni::Status rc = openni::STATUS_OK;

	openni::Device device;
	openni::VideoStream depth, color;
    openni::VideoFrameRef  depthFrame;

	const char* deviceURI = openni::ANY_DEVICE;
	//printf("%0x\n",deviceURI);
	std::cout << "deviceURI is " << static_cast<const void*>(deviceURI) << std::endl;
	if (argc > 1)
	{
		deviceURI = argv[1];
	}

	rc = openni::OpenNI::initialize();

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	rc = color.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = color.start();
	 	if (rc != openni::STATUS_OK)
	 	{
	 		printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
	 		color.destroy();
	 	}
	 }
	 else
	 {
	 	printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
	 }

	if (!depth.isValid() || !color.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}
	
	device.setDepthColorSyncEnabled(true);
	//device.setImageRegistrationMode(IMAGE_REGISTRATION_COLOR_TO_DEPTH);

	
	AXonCropping depthAC ;
	depthAC.originX = 0;
	depthAC.originY = 0;
	depthAC.width = 640;
	depthAC.height = 480;
	depthAC.gx = 0;
	depthAC.gy = 0;
	depth.setProperty(AXONLINK_STREAM_PROPERTY_CROPPING,depthAC);
	
    SampleViewer sampleViewer("AXon Depth Viewer", device, depth,color);
    rc = sampleViewer.init();
	if (rc != openni::STATUS_OK)
	{
		std::cout << "/* message */" << std::endl;
		openni::OpenNI::shutdown();
		return 3;
	}
	
	
	while (depth.isValid() && color.isValid())
	{
		length = sampleViewer.display();
		char key = static_cast<char>(waitKey(1));
		if(key == 'q' || key == 'Q') break;
		
	}
	
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;

	//SampleViewer sampleViewer("AXon Depth Viewer", device, depth);

	//rc = sampleViewer.init(argc, argv);
	//if (rc != openni::STATUS_OK)
	//{
	//	openni::OpenNI::shutdown();
	//	return 3;
	//}
	//sampleViewer.run();
}
