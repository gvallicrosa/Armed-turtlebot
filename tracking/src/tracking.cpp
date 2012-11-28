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

#include <omp.h>

std::vector<std::string>
&split(const std::string &s, char delim, std::vector<std::string> &elems) 
{
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> 
split(const std::string &s, char delim) 
{
  std::vector<std::string> elems;
  return split(s, delim, elems);
}
const double PI = 3.141592653589793;

// transformation parameters Cam2j4
float rotx= 0,roty=0,rotz=-PI/2,transx=-0.047,transy=0,transz=-0.08;
int debug=2;

int main(int argc, char **argv)
{
  // ROS initialisation --------------
  ros::init(argc, argv, "ball_tracking");
  ros::NodeHandle nodeh;
  std::string calibfile, circleString;
  int cameraid;
  nodeh.getParam("/visualservoing/calibfile", calibfile);
  nodeh.getParam("/visualservoing/cameraID",  cameraid);
  nodeh.getParam("/visualservoing/circleInit", circleString);
  unsigned int x[5];
  unsigned int y[5];
  std::vector<std::string> tokens = split(circleString, ',');
  unsigned int N = tokens.size()/2;
  for(int i = 0; i < N; ++i)
  {
    x[i] = atoi(tokens[2*i].c_str());
    y[i] = atoi(tokens[2*i + 1].c_str());
    std::cout << "(" << x[i] << ", " << y[i] << ")" << std::endl;
  }
   
  // Transform broadcaster
  static tf::TransformBroadcaster br;
  tf::Transform cam2j4;
  tf::Transform obj2cam;
  cam2j4.setRotation( tf::Quaternion(rotx, roty, rotz) );
  cam2j4.setOrigin(   tf::Vector3(transx, transy, transz) );
  obj2cam.setRotation( tf::Quaternion(0, 0, 0) );
  
  // Image containters ---------------
  vpImage<unsigned char> vpI, vpIshow;
  cv::Mat cvI;
  
  // Image grabber initialisation ---------------
  cv::VideoCapture capture(cameraid);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480 );
  capture.set(CV_CAP_PROP_FRAME_WIDTH, 640 );
  capture >> cvI;
  
  // Convert image ---------------
  vpImageConvert::convert(cvI, vpI);
  
  // Display initialisation ---------------
  //#if defined VISP_HAVE_X11
  //vpDisplayX d;
  //#elif defined VISP_HAVE_GDI
  //vpDisplayGDI d;
  //#elif defined VISP_HAVE_OPEN_CV
  vpDisplayOpenCV d;
  //#endif
  d.init(vpI, 0, 0, "") ;
  
  vpDisplay::display(vpI);
  vpDisplay::flush(vpI);

  vpMeEllipse ellipse; // VISP moving ellipse
  vpMe me;             // VISP moving edge

  //Set the tracking parameters ---------------
  me.setRange(15);
  me.setSampleStep(2);
  me.setPointsToTrack(350);
  me.setThreshold(8500);
  ellipse.setMe(&me);
  ellipse.setCircle(true);
  ellipse.setThresholdRobust(0.25); // default 0.2
  
  
//  for (int j = 0; j < 5; ++j)
//  {
//    std::cout << x[j] << ", " << y[j] << std::endl;
//    vpDisplay::displayCircle(vpI, 640-x[j],480-y[j],  3, vpColor::red, true);
//    vpDisplay::displayCircle(vpI, y[j], x[j], 3, vpColor::green, true);
//  }
//  vpDisplay::flush(vpI);
//  cv::waitKey(0);
  

  //Initialize the tracking and display ---------------
    // ask user for input
//  ellipse.initTracking(vpI);
    // or use ros parameters
  ellipse.initTracking(vpI, N, y, x);

  ellipse.setDisplay(vpMeSite::RANGE_RESULT) ; // uncomment to show search lines
  vpDisplay::flush(vpI);
//  cv::waitKey(0);
  //Load Camera parameters  --------------
  vpCameraParameters cam;
  std::string xmlName = calibfile;
  vpXmlParserCamera xmlParser;
  if( xmlParser.parse(cam, xmlName.c_str(), "LogitechC310", vpCameraParameters::perspectiveProjWithDistortion)  !=  vpXmlParserCamera::SEQUENCE_OK)
    std::cout << "Error reading the XML calibration data!" << std::endl;
  
  // Pose estimation ---------------
  vpPose pose;
  pose.clearPoint();
  vpPoint P[4];
  vpImagePoint IP[4];
  //double r = 0.068/2;
  double r = 0.043/2;
  double rI = 1;    // pixel radius
  P[0].setWorldCoordinates(0, 0, 0);
  P[1].setWorldCoordinates(r, 0, 0);
  P[2].setWorldCoordinates(0, r, 0);
  P[3].setWorldCoordinates(0, r, 0); // dealing with a sphere
  
  rI = ellipse.getA();  // current pixel radius
  IP[0] = ellipse.getCenter();
  IP[1] = vpImagePoint( IP[0].get_i() + rI, IP[0].get_j() );
  IP[2] = vpImagePoint( IP[0].get_i(), IP[0].get_j() + rI);
  IP[3] = vpImagePoint( IP[0].get_i(), IP[0].get_j() + rI);
  
  
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
  char centerStr[1024];
  char realVals[1024];
  vpTranslationVector t;
  
  double fps = 30.f;
  
  int tx = (int)((transx + 0.1 )* 100 / 0.2);
  int ty = (int)((transy + 0.1 )* 100 / 0.2);
  int tz = (int)((transz + 0.1 )* 100 / 0.2) ;
  int rx = (int)((rotx + PI/2 ) * 100/PI );
  int ry = (int)((roty + PI/2 ) * 100/PI );
  int rz = (int)((rotz + PI/2 ) * 100/PI );      
  
  cv::Mat filt;  

  while( true )
  {
     if (debug == 2){
        cv::namedWindow( "Camera Transformation", CV_WINDOW_NORMAL| CV_GUI_EXPANDED);
        
        cv::createTrackbar( "rotx", "Camera Transformation", &rx, 100, 0 );
        cv::createTrackbar( "roty", "Camera Transformation", &ry, 100, 0 );
        cv::createTrackbar( "rotz", "Camera Transformation", &rz, 100, 0 );
        rotx = (float) rx * PI/100 - PI/2;
        roty = (float) ry * PI/100 - PI/2;
        rotz = (float) rz * PI/100 - PI/2;
        
        cv::createTrackbar( "transx", "Camera Transformation", &tx, 100, 0 );
        cv::createTrackbar( "transy", "Camera Transformation", &ty, 100, 0 );
        cv::createTrackbar( "transz", "Camera Transformation", &tz, 100, 0 );
        transx = (float)tx*0.2/100 - 0.1;
        transy = (float)ty*0.2/100 - 0.1;
        transz = (float)tz*0.2/100 - 0.1;
        
        cam2j4.setRotation( tf::Quaternion(rotx, roty, rotz) );
        cam2j4.setOrigin(   tf::Vector3(transx, transy, transz) );
    }
  
  
    double t0 = omp_get_wtime();
    capture >> cvI;
    if(cvI.empty())
      break;

    cv::Canny(cvI, filt, 3, 100);    

    vpImageConvert::convert(filt, vpIshow);
    vpImageConvert::convert(cvI, vpI);
    vpDisplay::display(vpI);
    
    //Track the ellipse.
    try
      {ellipse.track(vpIshow);}
    catch(...)
      {
        // do stuff with ROS
        break;
      }
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
    
    sprintf(centerStr,"Center in pixels: (%f, %f), Radius in pixels: %f", IP[0].get_i(), IP[0].get_j(), rI);
    vpDisplay::displayCharString(vpI, vpImagePoint(20, 20), centerStr, vpColor::lightRed) ;
    
    if(t[2] < 0)
    {
        for(int i = 0; i < 3; ++i)
            t[i] = -t[i];
    }
        
    
    sprintf(realVals,"Center in world:  (%f, %f, %f), FPS: %lf, distance = %lf", t[0], t[1], t[2], fps, cv::sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]) );
    //tf::Vector3 vec_tf= cam2j4.getOrigin(); 
    //sprintf(realVals,"Cam2j4:  (%f, %f, %f), FPS: %lf, distance = %lf", vec_tf[0], vec_tf[1], vec_tf[2], fps, cv::sqrt(t[0]*t[0]   + t[1]*t[1] + t[2]*t[2]) );
    
    vpDisplay::displayCharString(vpI, vpImagePoint(40, 20), realVals, vpColor::lightRed) ;
    
    vpDisplay::flush(vpI);
    fps = 1/( omp_get_wtime() - t0 );
    
   
    
    obj2cam.setOrigin( tf::Vector3(t[0], t[1], t[2]) );
    br.sendTransform(tf::StampedTransform(cam2j4,  ros::Time::now(), "joint4",  "cam_pos"));
    br.sendTransform(tf::StampedTransform(obj2cam, ros::Time::now(), "cam_pos", "obj_pos"));
  }

  capture.release();
  return 0;
}
