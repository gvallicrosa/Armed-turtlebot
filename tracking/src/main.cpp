#include <ros/ros.h>
#include "tf/transform_broadcaster.h"

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpMeEllipse.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoint.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>
#include <visp/vpXmlParserCamera.h>

#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char **argv)
{
  // ROS initialisation --------------
  ros::init(argc, argv, "ball_tracking");
  ros::NodeHandle nodeh;
  std::string calibfile;
  int cameraid;
  nodeh.getParam("/calibfile", calibfile);
  nodeh.getParam("/cameraID",  cameraid);
  // Transform broadcaster
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setRotation( tf::Quaternion(0, 0, 0) );
  
  // Image containters ---------------
  vpImage<unsigned char> vpI;
  cv::Mat cvI;
  
  // Image grabber initialisation ---------------
  cv::VideoCapture capture(cameraid);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480 );
  capture.set(CV_CAP_PROP_FRAME_WIDTH, 640 );
  capture >> cvI;
  
  // Convert image ---------------
  vpImageConvert::convert(cvI, vpI);
  
  // Display initialisation ---------------
  #if defined VISP_HAVE_X11
  vpDisplayX d;
  #elif defined VISP_HAVE_GDI
  vpDisplayGDI d;
  #elif defined VISP_HAVE_OPEN_CV
  vpDisplayOpenCV d;
  #endif
  d.init(vpI, 0, 0, "") ;
  
  vpDisplay::display(vpI);
  vpDisplay::flush(vpI);

  vpMeEllipse ellipse; // VISP moving ellipse
  vpMe me;             // VISP moving edge

  //Set the tracking parameters ---------------
  me.setRange(20);
  me.setSampleStep(2);
  me.setPointsToTrack(300);
  me.setThreshold(7500);
  ellipse.setMe(&me);
  ellipse.setCircle(true);
  ellipse.setThresholdRobust(0.1); // default 0.2
  
  //Initialize the tracking and display ---------------
  ellipse.initTracking(vpI);
  //ellipse.setDisplay(vpMeSite::RANGE_RESULT) ; // uncomment to show search lines
  vpDisplay::flush(vpI);
  
  //Load Camera parameters  --------------
  vpCameraParameters cam;
  std::string xmlName = calibfile;
  vpXmlParserCamera xmlParser;
  if( xmlParser.parse(cam, xmlName.c_str(), "LogitechC110", vpCameraParameters::perspectiveProjWithDistortion)  !=  vpXmlParserCamera::SEQUENCE_OK)
    std::cout << "Error reading the XML calibration data!" << std::endl;
  
  // Pose estimation ---------------
  vpPose pose;
  pose.clearPoint();
  vpPoint P[4];
  vpImagePoint IP[4];
  double r = 0.034; // 3.4 cm
  double rI = 1;    // pixel radius
  P[0].setWorldCoordinates(0, 0, 0);
  P[1].setWorldCoordinates(-r, 0, 0);
  P[2].setWorldCoordinates(0, -r, 0);
  P[3].setWorldCoordinates(0, -r, 0); // dealing with a sphere
  
  rI = ellipse.getA();  // current pixel radius
  IP[0] = ellipse.getCenter();
  IP[1] = vpImagePoint( IP[0].get_i() + rI, IP[0].get_j() );
  IP[2] = vpImagePoint( IP[0].get_i(), IP[0].get_j() + rI);
  IP[3] = vpImagePoint( IP[0].get_i(), IP[0].get_j() - rI);
  
  
  for(int i = 0; i < 4; i++)
  {
    double x = 0, y = 0;               // the location in camera plane (meters!)
    vpPixelMeterConversion::convertPoint(cam, IP[i], x, y);
    P[i].set_x(x);
    P[i].set_y(y); // build the [X, Y, Z; x, y] vpPoint data structure
    pose.addPoint(P[i]); // add the 3D - 2D correspondences to the pose estimator
  }
  vpHomogeneousMatrix T;
  pose.computePose(vpPose::LAGRANGE, T); // robust initialisation
  pose.computePose(vpPose::VIRTUAL_VS, T); // refinement 
  pose.display(vpI, T, cam, 0.05, vpColor::cyan);
  vpDisplay::flush(vpI);
  
  // Strings to be displayed ---------------
  char centerStr[50];
  char realVals[50];
  vpTranslationVector t;
  
  while( true )
  {
    capture >> cvI;
    vpImageConvert::convert(cvI, vpI);
    vpDisplay::display(vpI);
    
    //Track the ellipse.
    ellipse.track(vpI);
    ellipse.display(vpI, vpColor::green) ;
    
    // ball has 3.42cm in radius
    pose.clearPoint();
    rI = ellipse.getA();                // current pixel radius
    IP[0] = ellipse.getCenter();
    IP[1] = vpImagePoint( IP[0].get_i() + rI, IP[0].get_j() );
    IP[2] = vpImagePoint( IP[0].get_i(), IP[0].get_j() + rI);
    IP[3] = vpImagePoint( IP[0].get_i(), IP[0].get_j() - rI);
    for(int i = 0; i < 4; i++)
    {
      double x = 0, y = 0;              // the location in camera plane (meters!)
      vpPixelMeterConversion::convertPoint(cam, IP[i], x, y);
      P[i].set_x(x);
      P[i].set_y(y);                    // build the [X, Y, Z; x, y] vpPoint data structure
      pose.addPoint(P[i]);              // add the 3D - 2D correspondences to the pose estimator
    }
    pose.computePose(vpPose::VIRTUAL_VS, T);
    T.extract(t);
    pose.display(vpI, T, cam, 0.05, vpColor::cyan);
    
    //std::sprintf(centerStr,"Center in pixels: (%f, %f), Radius in pixels: %f", IP[0].get_i(), IP[0].get_j(), rI);
    //vpDisplay::displayCharString(vpI, vpImagePoint(20, 20), centerStr, vpColor::lightRed) ;
    //std::sprintf(realVals,"Center in world:  (%f, %f, %f)", t[0], t[1], t[2]);
    //vpDisplay::displayCharString(vpI, vpImagePoint(40, 20), realVals, vpColor::lightRed) ;
    std::printf("Center in pixels: (%f, %f), Radius in pixels: %f\n", IP[0].get_i(), IP[0].get_j(), rI);
    std::printf("Center in world:  (%f, %f, %f)\n\n", t[0], t[1], t[2]);
    vpDisplay::flush(vpI);
    
    transform.setOrigin( tf::Vector3(t[0], t[1], t[2]) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "joint5", "obj_pos"));
  }
  capture.release();
  return 0;
}
