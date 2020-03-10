#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <sstream>
#include <vector>

using namespace std;


int main(int argc, char** argv)
{
    cv::Mat src_image = cv::imread("/home/reid/catkin_ws/temp_folder/img_depth_70_L10.png",CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
    cv::flip(src_image,src_image,1);
    cv::transpose(src_image, src_image);
    std::cout << "depth is " << src_image.depth()<< std::endl;
    std::cout << "type is " << src_image.type()<< std::endl;
    
    cv::Mat src_normalised;
    normalize(src_image, src_normalised, 0, 255, cv::NORM_MINMAX,CV_8UC1);
    cv::imshow("src_normalised",src_normalised);



    for (int i = 500; i < 520 ; i++)
    {
        ushort *ptr = src_image.ptr<ushort>(i);
        for (int j = 0; j < 350; j++)
        {
            //std::cout << i<< "_ptr_pre_"<<j << " is " <<ptr[j] << std::endl;
            if (ptr[j] == 0)
            {
                vector<ushort> tem_vector;
                //std::cout << "ptr_pre_"<<i << " is " <<ptr[i] << std::endl;
                cv::Mat cut_c(src_image, cv::Rect(j+1,i-2,5,5));
                
                for (int k = 0; k < cut_c.rows; k++)
                {
                    const ushort *ptr_cut_c = cut_c.ptr<ushort>(k);
                    for (int l = 0; l < cut_c.cols; l++)
                    {
                        std::cout <<i<<"_"<<j<< "_ptr_cut_c[l]_" << ptr_cut_c[l]<< std::endl;
                        if (ptr_cut_c[l] != 0)
                        {
                            tem_vector.push_back(ptr_cut_c[l]);
                        }
                       
                    }
                }
                std::cout << "tem_vector.size() is " << tem_vector.size()<< std::endl;
                if (tem_vector.size() != 0)
                {
                    int sum = accumulate(tem_vector.begin() , tem_vector.end() , 0);
                    ptr[j] = sum/tem_vector.size();
                }
                else
                {
                    continue;
                }
                
                //std::cout << "sum is "<< sum << std::endl;
                //std::cout << "tem_vector.size() is "<< tem_vector.size() << std::endl;
                
                
                //std::cout << "ptr_"<<i << " is " <<ptr[i] << std::endl;
            }
            //std::cout << i<< "_ptr_"<<j << " is " <<ptr[j] << std::endl;
        }
    }
    
    /*
    for (int i = 0; i < 350; i++)
    {
        ushort *ptr = src_image.ptr<ushort>(505);
        std::cout << "ptr_pre_"<<i << " is " <<ptr[i] << std::endl;
        if (ptr[i] == 0)
            {
                for (int k = 1; k < 480; k++)
                {
                    if (ptr[i+k] == 0)
                    {
                        continue;
                    }
                    else
                    {
                        ptr[i]=ptr[i+k];
                        break;
                    }
                }
            }
        //ushort *ptr = src_image.ptr<ushort>(505);
        std::cout << "ptr_"<<i << " is " <<ptr[i] << std::endl;
    }
    */
    /*
    for (int i = 2; i < 350; i++)
    {
        ushort *ptr = src_image.ptr<ushort>(505);
        //std::cout << "ptr_pre_"<<i << " is " <<ptr[i] << std::endl;
        if (ptr[i] == 0)
        {
            vector<ushort> tem_vector;
            std::cout << "ptr_pre_"<<i << " is " <<ptr[i] << std::endl;
            cv::Mat cut_c(src_image, cv::Rect(i-2,503,5,5));
            for (int j = 0; j < cut_c.rows; j++)
            {
                const ushort *ptr_cut_c = cut_c.ptr<ushort>(j);
                for (int k = 0; k < cut_c.cols; k++)
                {
                    if (ptr_cut_c[k] != 0)
                    {
                        tem_vector.push_back(ptr_cut_c[k]);
                    }
                }
            }
            int sum = accumulate(tem_vector.begin() , tem_vector.end() , 0);
            std::cout << "sum is "<< sum << std::endl;
             std::cout << "tem_vector.size() is "<< tem_vector.size() << std::endl;
            ptr[i] = sum/tem_vector.size();
            std::cout << "ptr_"<<i << " is " <<ptr[i] << std::endl;
        }
        //std::cout << "ptr_"<<i << " is " <<ptr[i] << std::endl;
    }
   
	*/
    normalize(src_image, src_image, 0, 255, cv::NORM_MINMAX,CV_8UC1);
    cv::imshow("src_image",src_image);
    cv::waitKey(0);



    return 0;
}