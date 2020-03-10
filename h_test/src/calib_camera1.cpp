#include <iostream> 
#include <sstream> 
#include <time.h>
#include <stdio.h> 
#include <fstream> 
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/calib3d/calib3d.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <string>
//#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

using namespace cv; 
using namespace std;

int main (int argc ,char **argv)
{
	int count = 1;
	Size image_size;
	Size board_corner_size = Size(7,4);
	vector<Point2f> img_corner;
	vector<vector<Point2f> > img_corner_seq;
 	for(;;)
	{	
		count++;
		stringstream ss;
		string str;
		ss << count;
		ss >> str;
		string filename = "/home/reid/Desktop/tem_folder/webcamera_undistort/calib_img_dsipc/img_"+str+".jpg";
		Mat img = imread(filename);	
		if(count == 2)
		{
			image_size.width = img.cols;
			image_size.height = img.rows;
			cout<<	image_size.width <<endl;
			cout<<	image_size.height <<endl; 	
		}
		if(0==findChessboardCorners(img,board_corner_size,img_corner))
		{
			cout << filename <<" ** cant find corner!" <<endl;
			cout << "\n" <<endl;
			continue;
		}
		else
		{
			Mat view_gray;
			cvtColor(img,view_gray,CV_RGB2GRAY);
			cornerSubPix(view_gray,img_corner,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			img_corner_seq.push_back(img_corner);
			drawChessboardCorners(view_gray,board_corner_size,img_corner,false);
			imshow("corner",view_gray);
			cout<<count<<endl;
			waitKey(5);
		}
		if(count == 339) break;
		
	}
	// calib  camera-------------------------------
	// board info 
	Size board_square_size = Size(70,70);
	vector<vector<Point3f> > object_point;
	vector<int> point_count;
		
	
	//intrinsic para
	double k[3][3] = {0};
	Mat camera_matrix = Mat(3,3,CV_32FC1,k);
	double dist1[1][5] = {0};
	Mat dist= Mat(1,5,CV_32FC1,dist1);
	vector<Mat> tvecsMat;
	vector<Mat> rvecsMat;
	int i,j,t;
	for(i=0;i<count;i++)
	{
		vector<Point3f> temPointSet;
		for(j=0;j<board_corner_size.height;j++)
		{
			for(t=0;t<board_corner_size.width;t++)
			{
				Point3f realPoint;
				realPoint.x = j * board_square_size.width;
				realPoint.y = t * board_square_size.height;
				realPoint.z = 0;
				temPointSet.push_back(realPoint);
			}
		}	
		object_point.push_back(temPointSet);
	}
	for(i=0;i<count;i++)
	{
		point_count.push_back(board_corner_size.height*board_corner_size.width);
	}
	calibrateCamera(object_point, img_corner_seq, image_size, camera_matrix, dist, rvecsMat, tvecsMat, 0);
	cout<< "matrix:"<< camera_matrix <<endl<<endl;
	cout<< "dist:"<< dist <<endl;	
	//cout<< "rvecsMat:"<<rvecsMat<<endl;
	//cout<< "tvecsMat:"<<tvecsMat<<endl;


	waitKey();
	
		
	return 1;



}



