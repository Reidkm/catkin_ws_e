#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    Mat frame;
    Mat grayframe;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID + apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    for (;;)
    {
        double inittime = (double)getTickCount();
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        Size patternsize(8,6); //interior number of corners
        
        vector<Point2f> corners; //this will be filled by the detected corners

        cvtColor(frame, grayframe, CV_BGR2GRAY);
        //CALIB_CB_FAST_CHECK saves a lot of time on images
        //that do not contain any chessboard corners
        bool patternfound = findChessboardCorners(grayframe, patternsize, corners,
                            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                            + CALIB_CB_FAST_CHECK);

        if(patternfound)
            cornerSubPix(grayframe, corners, Size(11, 11), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);

        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;
        double duration = ((double)getTickCount() - inittime) / getTickFrequency();    
         cout << "duration is " << duration << endl;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}