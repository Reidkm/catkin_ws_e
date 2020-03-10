#include<iostream>
#include<opencv2/opencv.hpp>

using namespace cv;

int main()
{
    VideoCapture capture;
    capture.open(0);
    if(!capture.isOpened())
    {
        printf("can not open ...\n");
        return -1;
    }

    Size size = Size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    VideoWriter writer;
    writer.open("/home/reid/Videos/a2.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, size, true);

    Mat frame, gray;
    namedWindow("output", CV_WINDOW_AUTOSIZE);

    for(;;)
    {
        capture.read(frame);
        //转换为黑白图像
        cvtColor(frame, gray, COLOR_BGR2GRAY);  
        //二值化处理 
        threshold(gray, gray, 0, 255, THRESH_BINARY | THRESH_OTSU);
        cvtColor(gray, gray, COLOR_GRAY2BGR);
        imshow("output", gray);
        writer.write(gray);
        char key = static_cast<char>(waitKey(10));
		if(key == 27) break;
		if(key=='w'||key=='W')
        break;
        //waitKey(10);
    }

    waitKey(0);
    capture.release();
    return 0;
}