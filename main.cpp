
// #include "disparity.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <stdio.h>

using namespace cv;
using namespace std;

// calcul the disparity map between two images
Mat calculDisp(Mat im1, Mat im2){
	Mat disp, disp8;
	cv::StereoSGBM sgbm(0,32,7,8*7*7);//,32*7*7,2,0,5,100,32,true);
	sgbm(im1, im2, disp);
/*

	
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = 16;
	sgbm.SADWindowSize = 7;
	sgbm.P1 = 8*7*7;
	sgbm.P2 = 32*7*7;
	sgbm.disp12MaxDiff = 2;
	sgbm.preFilterCap = 0;
	sgbm.uniquenessRatio = 5;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.fullDP = true;

	sgbm(im1, im2, disp);
*/
	disp.convertTo(disp8, CV_8U);
	//normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
	for(int u=0; u<disp.rows; u++){
		unsigned char *input = disp.ptr<unsigned char>(u);
		for(int v=0; v<disp.cols; v++) {
			printf("disp : \n %d", input[v]);
		}
	}

	return disp;
}


// Computation of the 3D coordinates and remove all the pixels 
// with a Z coodinate higher or lower than a threshold
Mat compute3DAndRemove(Mat disp){
	float thresMax = 2.5;
	float thresMin = 0.2;
	//float u0 = 258;
	float v0 = 156;
	float au = 410;
	float av = au;
	float b = 0.22;
	float z0 = 1.28;
	for(int u=0; u<disp.rows; u++){
		unsigned char *input = disp.ptr<unsigned char>(u);
		for(int v=0; v<disp.cols; v++) {
			int d = input[v]/16;
			//float x = (u-u0)*b/d - (b/2);
			//float y = au*b/d;
			float z = z0 - (((u-v0)*au*b)/(av*d));
			if(z<thresMin || z>thresMax){
				input[v] = 0;
			}
		}
	}
	return disp;
}

// Compute the v-disparity image
Mat computeVDisparity(Mat img){
	int maxDisp = 32;
	printf("height : %d", img.rows);
	Mat vDisp(312, maxDisp, CV_8UC1);
	for(int v=0; v<img.rows;v++){
		unsigned char *input = img.ptr<unsigned char>(v);
		//unsigned char *output = vDisp.ptr<unsigned char>(v);
		for(int u=0; u<img.cols; u++){
		
			if(input[u]>0){
				//printf("disp \n : %d", input[u]);
			//	output[input[u]] += 1;
			}
		}
	}
	return vDisp;
}




int main(int argc, char **argv)
{
	if(argc != 3) {
		cerr << "Usage : " << argv[0] << " image" << endl;
		return 0;
	}

	// Open image from input file in grayscale
	Mat img1 = imread(argv[1], 0);
	Mat img2 = imread(argv[2], 0);
	Mat disp, disp8;


	// Disparity map between the two images
	disp = calculDisp(img1, img2);
	disp.convertTo(disp8, CV_8U);
	imshow("left image", img1);
	imshow("right image", img2);
	imshow("diparity", disp8);

	// remove the road 
	//Mat imgRoad = compute3DAndRemove(disp8);
	//imshow("no road", imgRoad);

	// Compute VDisparity
	Mat vdisp = computeVDisparity(disp);
	//imshow("vdisp", vdisp);

	// Display images and wait for a key press


	waitKey();
	return 0;
}




