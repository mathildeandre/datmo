
// #include "disparity.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <stdio.h>

#include "tp_util.hpp"

using namespace cv;
using namespace std;

//2.1 calcul the disparity map between two images
Mat calculDisp(Mat im1, Mat im2){
    Mat disp, disp8;
    cv::StereoSGBM sgbm(0,32,7,8*7*7,32*7*7,2,0,5,100,32,true);
    sgbm(im1, im2, disp);
    disp.convertTo(disp8, CV_8U);
    return disp;
}


//2.2 Computation of the 3D coordinates and remove all the pixels with a Z coodinate higher or lower than a threshold
Mat compute3DAndRemove(Mat disp){
    float thresMax = 2.5;
    float thresMin = 0.2;
    //float u0 = 258; //unused here
    float v0 = 156;
    float au = 410;
    float av = 410;
    float b = 0.22;
    float z0 = 1.28;
    for(int u=0; u<disp.rows; u++){
        unsigned char *input = disp.ptr<unsigned char>(u);
        for(int v=0; v<disp.cols; v++) {
            int d = input[v]/16;
            //float x = (u-u0)*b/d - (b/2); //unused here
            //float y = au*b/d;             //unused here
            float z = z0 - (((u-v0)*au*b)/(av*d));
            if(z<thresMin || z>thresMax){
                input[v] = 0;
            }
        }
    }
    return disp;
}

//2.3.1 Compute the v-disparity image histogram of disparity matrix
Mat computeVDisparity(Mat img){
    int maxDisp = 32;
    Mat vDisp(img.rows, maxDisp, CV_8U, Scalar(0));
    for(int u=0; u<img.rows;u++){
        for(int v=0; v<img.cols; v++){
            int disp = (img.at<unsigned char>(u,v)) /8;
            if(disp>0 && disp <= maxDisp){
                vDisp.at<unsigned char>(u,disp) += 1;
            }
        }
    }
    return vDisp;
}

//2.3.3 Removing noise from v disparity map
Mat removeNoise(Mat img){
    Mat res = img.clone();
    int thresh = 60;
    //threshold( src_gray, dst, threshold_value, max_BINARY_value,threshold_type );
    threshold(img,res,thresh,255,3); // 255 > max binary value, 4 > zero inverted (to 0 under thresh)
    return res;
}

//3.3.3 Storing pixel of a map into an std::vector
std::vector<Point2f> storeRemainingPoint(Mat img){
    std::vector<Point2f> res;
    res.clear();
    for(int u=0; u<img.rows;u++){
        for(int v=0; v<img.cols; v++){
            int value = img.at<unsigned char>(u,v);
            if(value > 0){
                res.push_back(Point2f(u,v));
            }
        }
    }
    return res;
}

int main(int argc, char **argv)
{
    if(argc != 3) {
        cerr << "Usage : " << argv[0] << " image image" << endl;
        return 0;
    }

    // Open image from input file in grayscale
    Mat img1 = imread(argv[1], 0);
    Mat img2 = imread(argv[2], 0);
    //2.1.1 Displaying left and right loaded imgs
    //imshow("left image", img1);
    //imshow("right image", img2);

    // 2.1.2 Disparity map between the two images using SGBM
    Mat disp, disp8;
    disp = calculDisp(img1, img2);
    disp.convertTo(disp8, CV_8U);
    imshow("diparity map", disp8);

    //2.2.1 remove the road (z<0.2m) and upper pixels(z>2.5m)
    Mat imgRoad = compute3DAndRemove(disp8);
    imshow("Disparity map with height filter", imgRoad);

    //2.3.1 Compute VDisparity
    Mat vDispNoisy = computeVDisparity(disp);
    imshow("VDisparity method", vDispNoisy);

    //2.3.3 Removing noise from v disparity map
    Mat vDisp = removeNoise(vDispNoisy);
    imshow("VDisparity map no noise", vDisp);

    //2.3.4 extracting the remaining points and removing the floor
    std::vector<Point2f> tempVec = storeRemainingPoint(vDisp);
    //fitLineRansac(tempVec, line, iteration, sigma, a_max);
    Vec4f newline;
    fitLineRansac(tempVec, newline, 15, 2, 1);
    std::cout << newline << std::endl;

    // Display images and wait for a key press
    waitKey();
    return 0;
}




