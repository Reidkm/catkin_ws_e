// 2020/3/18  Reid

// C++ 标准库
#include <iostream>
#include <string>

#include <Eigen/Core> 
//Eigen lib

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>



// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 324.69931;
const double camera_cy = 239.38878;
const double camera_fx = 412.57083;
const double camera_fy = 414.97961;

using namespace std;
using namespace cv;

// 主函数 
int main( int argc, char** argv )
{
    // 读取./data/rgb.png和./data/depth.png，并转化为点云

    if (argc < 3 )
    {
        cout << "Usage : pnp  inputrgbimage  inputdepthimage" << endl;
        return -1;
    }

    // rgb image and d image path
    string inputrgbimage_path = argv[1];
    cout << "inputrgbimage_path is " <<  inputrgbimage_path << endl;
    string inputdepthimage_path = argv[2];
    cout << "inputdepthimage_path is " <<  inputdepthimage_path << endl;

    // 图像矩阵
    cv::Mat rgb, depth;
    // 使用cv::imread()来读取图像
    // API: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=imread#cv2.imread
    //rgb = cv::imread( "/home/reid/catkin_ws/img_rgb_70_L15.png" );
    // rgb 图像是8UC3的彩色图像
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    //

    depth = cv::imread( inputdepthimage_path, -1 );
    rgb = cv::imread( inputrgbimage_path );

    //camera intrinsic para 
    Mat K = (cv::Mat_<double>(3, 3) << 412.57083, 0, 324.69931, 0, 414.97961, 239.38878, 0, 0, 1.0);
    
    //camera distort para
    Mat distCoeffs =(cv::Mat_<double>(4, 1) <<-0.005948, -0.256112, 0.131517, 0);
    
    // checkerboard size
    Size board_size = Size(4,7);

    // image size
    Size image_size;  
    image_size.width = rgb.cols;
    image_size.height =rgb.rows;
   
    //vector for detected checerkerboard corners
    vector<Point2f> image_points_buf;
    vector<Point2f> image_points_buf_new;

    //find checkerboard corners
    bool found = findChessboardCorners( rgb, board_size, image_points_buf, CV_CALIB_CB_ADAPTIVE_THRESH );
    //
    if (found == 1 )
    {
        Mat view_gray;

        cvtColor(rgb,view_gray,CV_RGB2GRAY);

        //refine corners
        find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化
        
        //push corners to vector 
        image_points_buf_new.push_back(image_points_buf[0]);
        image_points_buf_new.push_back(image_points_buf[1]);
        image_points_buf_new.push_back(image_points_buf[4]);
        image_points_buf_new.push_back(image_points_buf[5]);
        
        //draw corners on image
        drawChessboardCorners(rgb,Size(2,2),image_points_buf_new,true); //用于在图片中标记角点
    }
    else
    {
        cout << "corners not found !" << endl;
        return -1 ;
    }


    // corners in world coordinate
    vector<Point3d> model_points;
    model_points.push_back(Point3d(0.0f, 0.0f, 0.0f));
    model_points.push_back(Point3d(100.0f, 0.0f, 0.0f));
    model_points.push_back(Point3d(0.0f, 100.0f, 0.0f));
    model_points.push_back(Point3d(100.0f, 100.0f, 0.0f));

    cv::Mat rotation_vector; // Rotation in axis-angle form

    cv::Mat translation_vector;
    
    //calculate transform matrix by pnp
    cv::solvePnP(model_points, image_points_buf_new, K, distCoeffs, rotation_vector, translation_vector);

    cout << "rotation_vector is "  << rotation_vector << endl;

    cout << "translation_vector is "  << translation_vector << endl;

    // test rotate vector and translation_vector by reprojection
    vector<Point3f> model_points_test;
    vector<Point2f> image_points_test;
    model_points_test.push_back(Point3f(200,0,0));

    // projectPoints func 
    projectPoints(model_points_test, rotation_vector, translation_vector, K, distCoeffs, image_points_test);

    for(int i=0; i < image_points_test.size(); i++)
    {
        circle(rgb, image_points_test[i], 3, Scalar(0,0,255), -1);

    }

    imshow("pnp test",rgb);
    if (cv::waitKey(0) > 0)
    {
      cv::destroyAllWindows();
    }
    
    //char key = static_cast<char>(cv::waitKey(0));

    // if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      
    //   cv::destroyAllWindows();

    // }

    cv::Mat rotation_mat;
    //tranform rotate vector  to matrix by rodrigues formula  in order to use in eigen
    Rodrigues(rotation_vector, rotation_mat);

    //Eigen::AngleAxisd rotate_axis;
    //Eigen::AngleAxisd rotate_axis;
    Eigen::Matrix3d r;
    cv::cv2eigen( rotation_mat, r );

    Eigen::Vector3d translation;
    cv::cv2eigen( translation_vector, translation );
    
    
    Eigen::Isometry3d Tcw=Eigen::Isometry3d::Identity();//如果没有直接初始化，先设为单位阵

    Tcw.prerotate(r);//然后添加旋转矩阵，或者向量，或者四元数

    Tcw.pretranslate(translation);//添加平移向量

    //Tcw is tranformation matrix which describe the relation world coordinate relative to camera coordinate
    cout << "Transform matrix = \n" << Tcw.matrix() <<endl;

    //the inverse of Tcw is is tranformation matrix which describe the relation camera coordinate relative to world coordinate
    Eigen::Matrix4d Tcw_inverse = Tcw.matrix().inverse();
    cout << "Tcw_inverse = \n" << Tcw_inverse<<endl;

    //the translation part of Tcw_inverse is the origin of camera coordinate  in world coordinate
    // the origin of camera coordinate mapping value in world camera coordinate is zero  if set Tcw_inverse(0,3) and Tcw_inverse(1,3) is zero
    
    Tcw_inverse(0,3) = 0;
    Tcw_inverse(1,3) = 0;
    
    cout << "Tcw_inverse = \n" << Tcw_inverse<<endl;
    //cout << "Tcw.inverse() = \n" << Tcw.inverse() <<endl;
    
    //T_1 is transformation matrix to rotate Tcw_inverse
    Eigen::Matrix4d T_1 ;
    T_1 << 0,-1,0,0,
           0,0,-1,0,
           -1,0,0,0,
           0,0,0,1;
    
    // final value of transformation matrix
    Tcw_inverse = T_1*Tcw_inverse;
    cout << "Tcw_inverse_rotated_changed = \n" << Tcw_inverse<<endl;



    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    //PointCloud::Ptr cloud ( new PointCloud );
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            //cout << "d is " << d << endl;
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            //PointT p;
            pcl::PointXYZRGB p;

            // 计算这个点的空间坐标
            //p.z = double(d) / camera_factor;
            p.z = double(d);
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    //pcl::visualization::CloudViewer viewer("pcd viewer");

	//viewer.showCloud(cloud);
    //pcl::io::savePCDFile( "./pointcloud_SF.pcd", *cloud );
    // 清除数据并退出
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("example"));
  //cout << " 1" << endl; 
    viewer->initCameraParameters ();
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
    // Eigen::Matrix4d Tcw_inverse ;

    // // Tcw_inverse << -0.995197 , -0.0263008 ,  0.0942904       ,    0,
    // // 0.0270535  , -0.999611 , 0.00671348   ,  455.577,
    // // -0.0940772, -0.00923212   ,-0.995522      ,     0,
    // //       0     ,      0   ,        0    ,       1;


    // Tcw_inverse <<
    // -0.999249,  -0.0385329, -0.00398308    ,       0,
    // 0.0386667  , -0.998364 , -0.0421165    , 471.716,
    // 0.0023537 ,  0.0422389  , -0.999105     ,      0,
    //       0   ,        0   ,        0  ,         1  ;                        //20200316
//    Tcw_inverse << 0.952642 , -0.130349  ,  0.27474     ,     -284.159,
//    0.303837 ,  0.445079 , -0.842371       ,   688.397,
//  -0.0124786 ,  0.885954 ,  0.463605 , -490.022,
//          0         ,0      ,   0    ,      1;

//     Tcw_inverse(0,3) = 0;
//     Tcw_inverse(1,3) = 0;

//     Eigen::Matrix4d T_1 ;
//     T_1 << 0,-1,0,0,
//            0,0,-1,0,
//            -1,0,0,0,
//            0,0,0,1;

//     Tcw_inverse = T_1*Tcw_inverse;
//     cout << "Tcw_inverse_changed = \n" << Tcw_inverse<<endl;



    pcl::transformPointCloud (*cloud, *cloud, Tcw_inverse);


    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> source_cloud_color_handler(cloud);

    viewer->addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  //viewer->addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer->addCoordinateSystem (1000.0, "cloud", 0);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position
    
    while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer->spinOnce ();
    }
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}
// Tcw_inverse = 
//   0.952642  -0.130349    0.27474          0
//   0.303837   0.445079  -0.842371          0
// -0.0124786   0.885954   0.463605  -0.980044
//         -0         -0         -0          1

// Tcw_inverse = 
//   0.952642  -0.130349    0.27474   -284.159
//   0.303837   0.445079  -0.842371    688.397
// -0.0124786   0.885954   0.463605   -490.022
//         -0         -0         -0          1
