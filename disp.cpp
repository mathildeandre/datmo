#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

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
	cv::StereoSGBM sgbm;

	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = 32;
	sgbm.SADWindowSize = 7;
	sgbm.P1 = 8*7*7;
	sgbm.P2 = 32*7*7;
	sgbm.disp12MaxDiff = 2;
	sgbm.preFilterCap = 0;
	sgbm.uniquenessRatio = 5;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.fullDP = true;

	sgbm(img1, img2, disp);
	//normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
	disp.convertTo(disp8, CV_8U);


	// Calculation of 3D coordinates
	for(int i=0; i<disp8.rows; i++){
		for(int j=0; j<disp8.cols; j++) {
			input[j];
		}
	}	

	// Display images and wait for a key press
	imshow("left image", img1);
	imshow("right image", img2);
	imshow("diparity", disp8);


	waitKey();
	return 0;
}




