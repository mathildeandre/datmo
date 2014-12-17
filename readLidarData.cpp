#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

Point mapToImage(float x, float y, int uo, int vo,
                 double camera_height,
                 double lidar_height,
                 double lidar_pitch_angle,
                 double camera_ty,
                 double alpha_u,
                 double alpha_v){
    double z=camera_height -(lidar_height + sqrt(x*x+y*y)*sin(lidar_pitch_angle));
    int u=(int)uo+alpha_u*(x/(y+camera_ty));
    int v=(int)vo+alpha_v*(z/(y+camera_ty));
    return Point(u,v);
}

int main3( int /*argc*/, char** /*argv*/ )
{
  //  Read lidar data from a file
  Mat lidar_data;
  string filename("lidarData.xml");
  FileStorage fs(filename, FileStorage::READ);
  fs["lidarData"]>> lidar_data;
  int nb_impacts = lidar_data.cols;
  int nb_frames = lidar_data.rows;
  //  extrinsic parameters of the lidar
  double lidar_pitch_angle = -1.*M_PI/180;
  double lidar_height = 0.47;

  //  parameters of the camera
  double uo = 256;
  double vo = 156;
  double alpha_u = 410;
  double alpha_v = 410;
  double camera_height = 1.28;
  double camera_ty = 1.8;

  //  define the parameters of the grid
  float x_min = -10.;
  float x_max = 10.;
  float y_min = 0.;
  float y_max = 30.;
  float x_step = 0.2;
  float y_step =  0.2;
  int nb_cells_x = (x_max-x_min)/x_step;
  int nb_cells_y = (y_max-y_min)/y_step;

  char key = 'a';
  int frame_nb = 0;

  //3.2.1 Kalman filter setup:
    KalmanFilter KF(4,2,0);
    KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
    Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));

    KF.statePre.at<float>(0) = 550; //prediction are done with integers (100.x, 100.y)
    KF.statePre.at<float>(1) = 1020;
    KF.statePre.at<float>(2) = 0.;
    KF.statePre.at<float>(3) = 0.;

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(0.1));
    setIdentity(KF.measurementNoiseCov, Scalar::all(0.1));
    setIdentity(KF.errorCovPost, Scalar::all(0.1));

    float ROI[4] = {4.,9.,7.5,11.}; //store the Region of interest x1,y1,x2,y2 size : 2.5x2

  while (key != 'q' && frame_nb != nb_frames)
  {
    //  Allocation/initialization of the grid
    Mat grid = Mat::zeros(Size(nb_cells_x, nb_cells_y), CV_32F);

    //  Read the stereo image
    ostringstream filename;
    filename<<"img/left_img_"<<frame_nb<<".png";
    Mat left_img = imread(filename.str(), 0);
    Mat left_display_img;
    cvtColor(left_img, left_display_img, CV_GRAY2RGB);

    //Computation of mean pos within the ROI
    float meanx = 0.;
    float meany = 0.;
    int nbVal = 0;

    //Display ROI
    Point inImg1 = mapToImage(ROI[0], ROI[1], uo, vo, camera_height, lidar_height, lidar_pitch_angle,camera_ty, alpha_u, alpha_v);
    Point  inImg2 = mapToImage(ROI[2], ROI[3], uo, vo, camera_height, lidar_height, lidar_pitch_angle,camera_ty, alpha_u, alpha_v);
    rectangle(left_display_img,inImg1,inImg2, Scalar(255,0,0));

    //  Process all the lidar impacts
    for (int i=0; i<nb_impacts/2; ++i)
    {
      double x=lidar_data.at<double>(frame_nb, 2*i);
      double y=lidar_data.at<double>(frame_nb, 2*i+1);

      //  compute the grid
      if (x>x_min && x<x_max && y>y_min && y<y_max && y>0)
        grid.at<float>((y_max-(y-y_min))/y_step, (x-x_min)/x_step) = 1.0;

      //  display on stereo image
      if (y>y_min && y < y_max &&  x>=x_min && x <= x_max) // 3.1.2 : affichage des points d'étude uniquement
      {
        Point p = mapToImage(x,y,uo, vo, camera_height, lidar_height, lidar_pitch_angle,camera_ty, alpha_u, alpha_v);
        int u= p.x;
        int v= p.y;
        if (u>0 && u<left_img.cols && v>0 && v<left_img.rows)
        {
          left_display_img.at<unsigned char>(v, 3*u) = 0;
          left_display_img.at<unsigned char>(v, 3*u+1) = 0;
          left_display_img.at<unsigned char>(v, 3*u+2) = 255;
        }
        //Computation of mean pos within the ROI
        if( x>ROI[0] && x<ROI[2] && y>ROI[1] && y<ROI[3])  {
          meanx += x;
          meany += y;
          nbVal ++;
        }
      }
    }

    //Kalman prediction (internal update)
    Mat prediction = KF.predict();
    Point predictPt1(prediction.at<float>(0),prediction.at<float>(1));

    //Computation of mean pos within the ROI
    meanx /= nbVal;
    meany /= nbVal;
    measurement(0) = meanx*100;
    measurement(1) = meany*100;

    //Maj de la ROI
    ROI[0] = meanx-1.5;
    ROI[1] = meany-1.;
    ROI[2] = meanx+1.5;
    ROI[3] = meany+1.;


    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0), estimated.at<float>(1));

    std::cout << meanx << ", " << meany << std::endl;
    std::cout << ROI[0] << ", " << ROI[1] << std::endl;
    std::cout << ROI[2] << ", " << ROI[3] << std::endl;
    std::cout << (float)statePt.x/100. << ", " << (float)statePt.y/100. << std::endl;


    //Draw Kalman rectangles prevision
    inImg1 = mapToImage((float)statePt.x/100.-1.5, (float)statePt.y/100.-1, uo, vo, camera_height, lidar_height, lidar_pitch_angle,camera_ty, alpha_u, alpha_v);
    inImg2 = mapToImage((float)statePt.x/100.+1.5, (float)statePt.y/100.+1, uo, vo, camera_height, lidar_height, lidar_pitch_angle,camera_ty, alpha_u, alpha_v);
    rectangle(left_display_img,inImg1,inImg2, Scalar(0,255,255));

    //   prepare the display of the grid
    Mat display_grid; //  to have a RGB grid for display
    grid.convertTo(display_grid, CV_8U, 255);
    cvtColor(display_grid, display_grid, CV_GRAY2RGB);

    Mat display_grid_large;// to have a large grid for display
    resize(display_grid, display_grid_large, Size(600,600));

    //  show images
    // imshow("top view",  display_grid_large);
    imshow("left image", left_display_img);

    //  Wait for the user to press a key
    frame_nb++;
    key = waitKey( );
  }
  return 0;
}
