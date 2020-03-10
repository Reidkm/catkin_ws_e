#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <sstream>


using namespace std;
using namespace cv;
void cameraPoseFromHomography(const Mat& H, Mat& pose)
{
    pose = Mat::eye(3, 4, CV_64FC1); //3x4 matrix
    float norm1 = (float)norm(H.col(0)); 
    float norm2 = (float)norm(H.col(1));
    float tnorm = (norm1 + norm2) / 2.0f;

    Mat v1 = H.col(0);
    Mat v2 = pose.col(0);

    cv::normalize(v1, v2); // Normalize the rotation

    v1 = H.col(1);
    v2 = pose.col(1);

    cv::normalize(v1, v2);

    v1 = pose.col(0);
    v2 = pose.col(1);

    Mat v3 = v1.cross(v2);  //Computes the cross-product of v1 and v2
    Mat c2 = pose.col(2);
    v3.copyTo(c2);      

    pose.col(3) = H.col(2) / tnorm; //vector t [R|t]
}


/*
  firt tag_image
Homography: [[ 6.56823500e-01 -3.74605557e-02 -1.82336297e+01]
                    [ 4.08351830e-03  7.14774850e-01 -1.06904522e+01]
                    [-1.02979442e-04  8.88486526e-06 -4.53393924e-02]]
           Center: [402.15866863 235.78728486]
          Corners: [[416.684021   252.16566467]
                    [385.89352417 250.8433075 ]
                    [387.69348145 219.47674561]
                    [418.50427246 220.65678406]]


    second tag_image
       Homography: [[ 4.72997892e-01 -1.04409137e+00  2.86624804e+01]
                    [ 1.08276828e+00 -2.07884700e-01  2.31340790e+01]
                    [ 9.76577675e-04 -8.13727834e-04  6.34983246e-02]]
           Center: [451.38955339 364.32581677]
          Corners: [[461.56713867 351.44909668]
                    [462.24847412 374.10391235]
                    [441.26403809 377.13665771]
                    [439.90054321 353.98034668]]
      corner_1:[461.567, 351.449]
corner_2:[462.248, 374.104]
corner_3:[441.264, 377.137]
corner_4:[5.46414e-38, 0]
object_point_11:[88.7, 0, 0]
object_point_12:[88.7, 88.7, 0]
object_point_13:[0, 88.7, 0]
object_point_14:[8.96831e-44, 0, 4.62428e-44]
rvec:[-0.2327231031130478;
 0.0280245140376717;
 -0.04692759290730396]
tvec:[464.2195040440504;
 370.1188451659016;
 1228.571756468026]
rotation_matrix:[0.9985133141407325, 0.04323642591746341, 0.03319296548328336;
 -0.04972737739349316, 0.9719528307450139, 0.2298583971585971;
 -0.02232374120204422, -0.231167269051304, 0.9726578762844139]





     third tag_image
 Homography: [[ 2.47848665e-02 -7.41688564e-01  1.92924129e+01]
                    [ 6.86043404e-01 -4.27219779e-04  2.07881956e+01]
                    [-5.37045596e-05 -1.15680711e-04  6.04409794e-02]]
           Center: [319.19424744 343.94207069]
          Corners: [[330.13027954 331.66900635]
                    [331.53564453 354.93582153]
                    [308.19674683 356.28411865]
                    [306.82751465 332.92575073]]

corner_1:[306.828, 332.926]
corner_2:[330.13, 331.669]
corner_3:[331.536, 354.936]
corner_4:[308.197, 356.284]
object_point_1:[0, 0, 0]
object_point_2:[88.7, 0, 0]
object_point_3:[88.7, 88.7, 0]
object_point_4:[0, 88.7, 0]
rvec:[-0.2917945686306199;
 -0.09603351742829513;
 -0.04493437413121974]
tvec:[-36.24000707758898;
 293.9346766281218;
 1307.674033654362]
rotation_matrix:[0.9944242347293704, 0.05811485897544624, -0.08799491206410878;
 -0.03031715521104985, 0.9567673462456621, 0.2892699695058969;
 0.1010015419823646, -0.2849893126492595, 0.9531945133040198]




forth tag_img
Homography: [[-8.26940335e-01  4.58221511e-03 -9.63732288e+00]
                    [-1.13430445e-01 -7.07824401e-01 -8.15608186e+00]
                    [-3.74488061e-04  6.90174540e-06 -2.92639809e-02]]
           Center: [329.32371363 278.70718883]
          Corners: [[305.05413818 253.83190918]
                    [353.13592529 255.07145691]
                    [352.9911499  302.96530151]
                    [304.88262939 302.96713257]]





*/


int main(int argc, char** argv)
{       
	Mat img1 = imread("/home/reid/catkin_ws_1/src/h_test/image/apriltag_3.jpg");
	cout<< "img_channel:"<<img1.channels()<<endl;
	cout<< "img_height:"<<img1.rows<<endl;
  	cout<< "img_width:"<<img1.cols<<endl;
	for(int i= 0;i<img1.rows;i++)
	{
		Vec3b*data = img1.ptr<Vec3b>(i);
		data[320][0] = 0;
		data[320][1] = 0;
		data[320][2] = 255;
		

	}
	for(int i= 0;i<img1.cols;i++)
	{
		Vec3b*data = img1.ptr<Vec3b>(240);
		data[i][0] = 0;
		data[i][1] = 0;
		data[i][2] = 255;
		

	}
	putText(img1, "2", Point2f(330.13027954 ,331.66900635), FONT_HERSHEY_COMPLEX, 0.8, (255,255,255), 1);
        //putText(img1, "2", Point2f(451.56713867, 351.44909668), FONT_HERSHEY_COMPLEX, 1, (255,255,255), 1);
	putText(img1, "3", Point2f(331.53564453, 354.93582153), FONT_HERSHEY_COMPLEX, 0.8, (255,255,255), 1);
	putText(img1, "4", Point2f(308.19674683, 356.28411865), FONT_HERSHEY_COMPLEX, 0.8, (255,255,255), 1);
	putText(img1, "1", Point2f(306.82751465, 332.92575073), FONT_HERSHEY_COMPLEX, 0.8, (255,255,255), 1);
	imshow("april_win",img1);
	






    cout<<"\n" <<endl;
    cout<< "=============================================solpnp======================="<<endl;   
    vector<Mat> r, t, n;
    double dist_coeff[5] =  {-0.2917102008989657, 0.06272372294560866, 0.001404208147185992, -0.002374556830271888, 0};
    Mat m_dist_coeff(5,1 ,CV_64FC1,dist_coeff);
    double intrinsic[9] = { 350.5370881773286, 0, 316.5051921872953, 0, 350.9985064348879, 255.105363895103, 0, 0, 1};
    Mat mIntrinsic(3, 3, CV_64FC1, intrinsic);
    //double corners[8] ={ 365.45080566, 191.01457214 ,519.95574951, 188.33111572 ,512.70233154, 302.56768799, 377.36868286, 305.65625};
    //Mat m_corners(2, 4, CV_64FC1, corners);
    //double object_point[12] = { 0,0,0, 173 ,0,0 ,173,173,0,0,173,0}; 
    //Mat m_object_point(4,3,CV_16UC1,object_point);
    //float rvec[3] = {0,0,0};
    //Mat m_rvec(3,1,CV_64FC1,rvec);
    //float tvec[] = {0,0,0};
    // m_tvec(3,1,CV_64FC1,tvec);
    vector<Point2f> corners;
    corners.push_back(Point2f(305.05413818 ,253.83190918));
    corners.push_back(Point2f(353.13592529 ,255.07145691));
    corners.push_back(Point2f(352.9911499 , 302.96530151));
    corners.push_back(Point2f(304.88262939 ,302.96713257));
    for(int i=0;i<4 ;i++)
    {
	cout << "corner_"<<i+1<<":"<<corners[i]<<endl;

    }
    vector<Point3f> object_point;
    object_point.push_back(Point3f(0,0,0));
    object_point.push_back(Point3f(173 ,0,0));
    object_point.push_back(Point3f(173,173,0));
    object_point.push_back(Point3f(0,173,0));
    for(int i=0;i<4 ;i++)
    {
	cout << "object_point_"<<i+1<<":"<<object_point[i]<<endl;

    }
    Mat m_rvec,m_tvec;
    solvePnP(object_point,corners,mIntrinsic,m_dist_coeff,m_rvec,m_tvec);
    cout << "rvec:"<<m_rvec<<endl;
    cout << "tvec:"<<m_tvec<<endl;
    Mat rvec_decomp;
    Rodrigues(m_rvec, rvec_decomp);
    cout << "rotation_matrix:" <<rvec_decomp<<endl;
    cout<< "=============================================solpnp======================="<<endl;
    cout<<"\n" <<endl;
    //waitKey(0);




    VideoCapture capture(0);
    int count = 0;
    for(;;)
	{	
		stringstream ss;
		string str;
		ss << count;
		ss >> str;
		string filename = "/home/reid/Desktop/tem_folder/webcamera_undistort/calib_img_dsipc/img_2"+str+".jpg";
		Mat image;
		capture >> image;
		namedWindow("img_calib",0);
		imshow("img_calib", image);
		char key = static_cast<char>(waitKey(1));
		if(key =='w'|| key =='W' )
		{
			imwrite(filename,image);
			count++;	
			continue;
		}
		
	}



    /*
    for (int i = 0; i< 9;i++)
    {
         cout<< "intrinsic"<<i<<":"<<intrinsic[i]<<endl;
    }
    cout<< "mIntrinsic:"<<mIntrinsic<<endl;
    //tag front 50cm
    //double homo[9] = { 9.69611391e-02 , 6.83448220e-01 ,-2.04023713e+01,-6.53397667e-01 ,-8.12010968e-03 ,-1.38901161e+01,2.25592191e-04 , -2.95613745e-05, -4.95306411e-02};
    //tag front 25cm 
    //double homo[9] = { -4.86144987e-03 , 7.76051052e-01 ,-1.04716630e+01,-7.26697502e-01 , 6.00042517e-02 ,-6.28129161e+00,-2.08431737e-05 , 1.95063037e-04 ,-2.43127365e-02};
//tag front 50cm right 20cm 
    double homo[9] = { -7.14957271e-02, -5.72841045e-01 , 2.25776891e+01,6.31254415e-01 , 7.95883391e-02,  6.70842650e+00,-3.58129855e-04 , 3.81868606e-04 , 5.46046895e-02};

    
    Mat mhomo(3, 3, CV_64FC1, homo);
    for (int i = 0; i< 9;i++)
    {
         cout<< "homo"<<i<<":"<<homo[i]<<endl;
    }
    cout<< "mhomo:"<<mhomo<<endl;
    
    decomposeHomographyMat(mhomo, mIntrinsic, r, t, n); 
    cout << "========Homography========" << endl; 
    
    for(int i=0; i<r.size(); ++i) 
    {    cout << "======== " << i << " ========" << endl; 
         cout << "rotation" << i << " = " << endl;
         cout << r.at(i) << endl; 
         cout << "translation" << i << " = " << endl; 
         cout << t.at(i) << endl;
         cout << "normal"<<i<<"="<<endl; 
         cout << n.at(i)<<endl;
    }
    

    //Mat H_img_0612 = ( cv::Mat_<unsigned char>(3, 3) << -7.87461995e-01, -2.32249925e-01, -5.29864701e+00, 5.23535392e-02, -8.56628524e-01, -6.63047861e+00, -5.42578228e-06, -6.29017167e-04, -1.71504362e-02 );
    //double homo_img_0612[] = { -7.87461995e-01, -2.32249925e-01, -5.29864701e+00, 5.23535392e-02, -8.56628524e-01, -6.63047861e+00, -5.42578228e-06, -6.29017167e-04, -1.71504362e-02 }; 
    //CvMat Mat_homo_img_0612=cvMat(3, 3, CV_64FC1, homo_img_0612);
    //cout <<"homo_cvmat_type:" <<Mat_homo_img_0612 <<endl;
    //Mat k = ( cv::Mat_<unsigned char>(3, 3) << 358.088456, 0.000000, 410.437559, 0.000000, 356.402300, 228.795528, 0.000000, 0.000000, 1.000000 );
    //cout<<"k:" <<k <<endl;
    //Mat rotation;
    //Mat translation;	
    //Mat normal;
    //int result = decomposeHomographyMat(H_img_0612,k,rotation,translation,normal);
    






    
    cout<<"-------------------------------------------------------------------------------"<<endl;
    double h[3][3] = {2, 6, 3, 6, 4.55, 6.1, 4.1, 2.9, 3.1};
    Mat h3_3 = Mat(3,3,CV_32F ,h);
    cout<< h3_3 <<endl;	
    Mat h3_3_out;
    cameraPoseFromHomography(h3_3, h3_3_out);
    cout<< "h3_3_out:"<< h3_3_out<<endl;


    cout<<"--------------img_0612--------------"<<endl;
    Mat H_img_0612 = ( cv::Mat_<unsigned char>(3, 3) << -7.87461995e-01, -2.32249925e-01, -5.29864701e+00, 5.23535392e-02, -8.56628524e-01, -6.63047861e+00, -5.42578228e-06, -6.29017167e-04, -1.71504362e-02 );  
    Mat  Pose_0612;
    cout<< "img_0612_H:"<<H_img_0612 <<endl;
    cameraPoseFromHomography(H_img_0612, Pose_0612);   
    cout<< "img_0612_pose:"<< Pose_0612<<endl;
        cout<<"--------------img_0614--------------"<<endl;

    Mat H_img_0614 = ( cv::Mat_<unsigned char>(3, 3) << -7.88517290e-01, -2.94322900e-01, -4.42042115e+00, 2.20092998e-02, -8.47105560e-01, -5.49853765e+00, -2.30852673e-05, -6.26340473e-04, -1.46588025e-02 );   
    Mat  Pose_0614;
    cout<< "img_0612_H:"<<H_img_0614 <<endl;
    cameraPoseFromHomography(H_img_0614, Pose_0614);   
    cout<< "img_0612_pose:"<< Pose_0614<<endl;
    */
	
    return 0;
}

