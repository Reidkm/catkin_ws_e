#include <opencv2/opencv.hpp> 
#include <iostream>
#include <sstream>
#include <string>

using namespace std; 
using namespace cv;

int main(int argc, const char** argv) {

    if (argc < 3)
    {
        cout << "Usage : test_image_bit_operate rgbimage depthimage" << endl;
    }

    string rgbimage_path = argv[1];
    string depthimage_path = argv[2];
    
    Mat rgb = imread(rgbimage_path,-1);
    Mat depth = imread(depthimage_path,-1);

    //rgb.convertTo(rgb, CV_16U);

    // vector<Mat> depth_vector_to_merge;
    // depth_vector_to_merge.push_back(depth);
    // depth_vector_to_merge.push_back(depth);
    // depth_vector_to_merge.push_back(depth);

    // merge(depth_vector_to_merge, depth);

    cout << " rgb  dims:" << rgb.dims << "\n"
         << "rows:" << rgb.rows << "\n"
         << "cols:" << rgb.cols << "\n"
         << "channels:" << rgb.channels() << "\n"
         << "type:" << rgb.type() << "\n"
         << "depth:" << rgb.depth() << "\n"
         << "size:" << rgb.size << "\n"
         << "----------------"<< "\n"
         << " dept  dims:" << depth.dims << "\n"
         << "rows:" << depth.rows << "\n"
         << "cols:" << depth.cols << "\n"
         << "channels:" << depth.channels() << "\n"
         << "type:" << depth.type() << "\n"
         << "depth:" << depth.depth() << "\n"
         << "size:" << depth.size << endl;
    
    namedWindow("rgb",WINDOW_AUTOSIZE);
    namedWindow("depth",WINDOW_AUTOSIZE);

    imshow("rgb", rgb);
    imshow("depth", depth);

    Size image_size(rgb.cols,rgb.rows);

    Mat depth_8unit3C(image_size,CV_8UC3);


    for(int row=0; row < image_size.height; row++)
    {
        ushort *ptr_depth = depth.ptr<ushort>(row);
        	//Vec3b* data;
        Vec3b *ptr_depth_8unit3C = depth_8unit3C.ptr<Vec3b>(row);
        for(int col=0; col < image_size.width; col++)
        {   
            ptr_depth_8unit3C[3*col+ 0]  = (ptr_depth[col])&0xFF ;
            ptr_depth_8unit3C[3*col+ 1]  = (ptr_depth[col]>>8)&0xFF ;
            ptr_depth_8unit3C[3*col+ 2]  = 0 ;
            //n_left = (num  >> 8) & 0xff; //取高8位 n_right =  2 ^7 -1 = 127
            //n_right = num &0xFF; //取低8位 n_right =  2 ^8 -1 = 255
            // uchar bValue = pValue[3*col  + 0];
			// 	uchar gValue = pValue[3*col + 1];
			// 	uchar rValue = pValue[3*col + 2];
                // const uchar *uc_pixel = ptr;
                // int a = uc_pixel[0];

                //把高8位与低8位结合
    // num_three = n_left;
    // num_three <<= 8; //恢复高位
    // num_three |= n_right; //或上低位
        // int b = uc_pixel[1];
        // int c = uc_pixel[2];
        // 不使用中间指针
                    //int a = ptr[0];
            //int b = ptr[1];
            //int c = ptr[2];
                    //sum += a + b + c;
                    //ptr += 3;
            //cout<< ptr[
        }
    }


        // char key = static_cast<char>(waitKey(1));
        // if(key == 27 key=='q'||key=='Q') break;
        // else if (key=='w'||key=='W')
        // {
        
        // }

    
    


    
    
    
    // Mat combined_image;
    // hconcat(rgb,depth,combined_image);
    // imshow("combined_image",combined_image);
    imshow("depth_8unit3C",depth_8unit3C);
    waitKey();
    destroyAllWindows();

    return 0;
}