//#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <string>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
 
using namespace cv;
using namespace std;


// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(0,1); 
    T(2,3) = tvec.at<double>(0,2);
    return T;
}



int main( int argc, char* argv[] )
{
    if (argc < 2 )
    {
        cout << "Usage : pnp  inputimage" << endl;
        return -1;
    }
    
    string input_image_path = argv[1];
    cout << "input_image_path is " <<  input_image_path << endl;

    Mat K = (cv::Mat_<double>(3, 3) << 412.57083, 0, 324.69931, 0, 414.97961, 239.38878, 0, 0, 1.0);

    Mat distCoeffs =(cv::Mat_<double>(4, 1) <<-0.005948, -0.256112, 0.131517, 0);

    Size board_size = Size(4,7);

    Size image_size;  
   
    vector<Point2f> image_points_buf;
    vector<Point2f> image_points_buf_new;

    //Mat imageInput = imread(argv[1]);

    Mat imageInput = imread(input_image_path);
   
    image_size.width = imageInput.cols;
    
    image_size.height =imageInput.rows;
   
   
   //vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */

    bool found = findChessboardCorners( imageInput, board_size, image_points_buf, CV_CALIB_CB_ADAPTIVE_THRESH );
  
    //
    if (found == 1 )
    {
        Mat view_gray;

        cvtColor(imageInput,view_gray,CV_RGB2GRAY);

        find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化

        image_points_buf_new.push_back(image_points_buf[0]);
        image_points_buf_new.push_back(image_points_buf[1]);
        image_points_buf_new.push_back(image_points_buf[4]);
        image_points_buf_new.push_back(image_points_buf[5]);

        drawChessboardCorners(imageInput,Size(2,2),image_points_buf_new,true); //用于在图片中标记角点
    }
    else
    {
        cout << "corners not found !" << endl;
        return -1 ;
    }
    
    
   
    


    
/*
    //for(vector<Point2f>::iterator iter=image_points_buf.begin();iter!=image_points_buf.end();++iter)
    {
        
       // cout<<*iter<<endl;
    }
*/
    //drawChessboardCorners(imageInput,board_size,image_points_buf,true); //用于在图片中标记角点


    

    
    
    //image_points_buf_new[0] = image_points_buf[0] ;
    //image_points_buf_new[1] = image_points_buf[1] ;
    //image_points_buf_new[2] = image_points_buf[4] ;
    //image_points_buf_new[3] = image_points_buf[5] ;
    //cout << "1 : " << image_points_buf[0] << endl;
    //cout << "2 : " << image_points_buf[1] << endl;
    //cout << "4 : " << image_points_buf[4] << endl;
    //cout << "5 : " << image_points_buf[5] << endl;


    


    vector<Point3d> model_points;

    model_points.push_back(Point3d(0.0f, 0.0f, 0.0f));
    model_points.push_back(Point3d(100.0f, 0.0f, 0.0f));
    model_points.push_back(Point3d(0.0f, 100.0f, 0.0f));
    model_points.push_back(Point3d(100.0f, 100.0f, 0.0f));
/*
    for (int i = 0; i < image_points_buf_new.size(); i++)
    {
        cout << "------" << endl;
        cout << image_points_buf_new[i] <<endl;
        cout << model_points[i] <<endl;
    }
*/
    cv::Mat rotation_vector; // Rotation in axis-angle form

	cv::Mat translation_vector;

    cv::solvePnP(model_points, image_points_buf_new, K, distCoeffs, rotation_vector, translation_vector);

    cout << "rotation_vector is "  << rotation_vector << endl;

    cout << "translation_vector is "  << translation_vector << endl;

    cv::Mat rotation_mat;

    Rodrigues(rotation_vector, rotation_mat);

    //Eigen::AngleAxisd rotate_axis;
    //Eigen::AngleAxisd rotate_axis;
    Eigen::Matrix3d r;
    Eigen::Vector3d translation;

    cv::cv2eigen( translation_vector, translation );
    cv::cv2eigen( rotation_mat, r );
    

    Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();//如果没有直接初始化，先设为单位阵

    Tcw.prerotate(r);//然后添加旋转矩阵，或者向量，或者四元数

    Tcw.pretranslate(translation);//添加平移向量

    cout << "Transform matrix = \n" << Tcw.matrix() <<endl;

    Eigen::Matrix4d Tcw_inverse = Tcw.matrix().inverse();
    cout << "Tcw_inverse = \n" << Tcw_inverse<<endl;
    Tcw_inverse(0,3) = 0;
    Tcw_inverse(1,3) = 0;

    cout << "Tcw_inverse = \n" << Tcw_inverse<<endl;
    //cout << "Tcw.inverse() = \n" << Tcw.inverse() <<endl;
    
    Eigen::Matrix4d T_1 ;
    T_1 << 0,-1,0,0,
           0,0,-1,0,
           -1,0,0,0,
           0,0,0,1;
    
    Tcw_inverse = T_1*Tcw_inverse;
    cout << "Tcw_inverse_changed = \n" << Tcw_inverse<<endl;




     //Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();

//     //Tcw =  cvMat2Eigen( rotation_vector, translation_vector );
    //cout << "rotation_vector type is "<< rotation_vector.type()<< endl;

     //Eigen::Isometry3f Tcw(rotation_vector);//rotation可以是旋转矩阵，可以是四元数，可以是旋转向量
    //Tcw.rotate
//     //Tcw.rotate ( rotation_vector ); 

//     //Tcw.pretranslate(Eigen::Vector3d(translation_vector));

//     cout << "Transform matrix = \n"  << "================="<< Tcw.matrix() <<endl;


//     //Rodrigues(rotation_vector, rotation_mat);

//     cout << "rotation_mat is "  << rotation_mat << endl;

//     cout << "rotation_vector type  is "  << rotation_vector.type() << "\n"
//          << "-----------------------" << endl;

//     Mat transform_1_inverse;

//     Mat transform_1 =(cv::Mat_<double>(4, 4) <<0.9526423065876992, 0.3038369944752536, -0.01247864125266401,0.05542674085009167,
//  -0.1303488357928504, 0.445078508732422, 0.885953893874829,0.09070649888630543,
//  0.2747395433983185, -0.8423705846317096, 0.463605415672116,0.8851317611516632,
//  0,0,0,1);

//     cout << "transform_1 is "  << transform_1 << endl;

//     invert(transform_1, transform_1_inverse, DECOMP_LU);

//     cout << "transform_1_inverse is "  << transform_1_inverse << endl;

//     transform_1_inverse = (cv::Mat_<double>(4, 4) <<0.9526423065876993, -0.1303488357928504, 0.2747395433983184, 0,
//  0.3038369944752536, 0.4450785087324219, -0.8423705846317094, 0,
//  -0.01247864125266402, 0.885953893874829, 0.4636054156721159, -0.4900220035265113,
//  0, 0, 0, 1);

//     cout << "transform_1_inverse_no_t is "  << transform_1_inverse << endl;

    // Project a 3D point (10, 0, 0.0) onto the image plane

    vector<Point3f> model_points_test;

    vector<Point2f> image_points_test;

    model_points_test.push_back(Point3f(200,0,0));

	     
    projectPoints(model_points_test, rotation_vector, translation_vector, K, distCoeffs, image_points_test);

    for(int i=0; i < image_points_test.size(); i++)
    {
        circle(imageInput, image_points_test[i], 3, Scalar(0,0,255), -1);

    }

    imshow("pnp test",imageInput);

    char key = static_cast<char>(cv::waitKey(0));

    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      
      cv::destroyAllWindows();

    }

    return 0;
}
