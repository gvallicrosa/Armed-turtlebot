//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <std_msgs/String.h>
#include <sstream>

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace std;

// Global variables
int low_hue, high_hue, low_sat, high_sat, debug;
cv::Rect prev_roi;
bool Never_detected = true;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

//Declare publisher of type string

ros::Publisher pub1;
ros::Publisher pub2;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    // OUTPUT PARAMETERS PUBLISHED
	cv::Point centroid = cv::Point(-1,-1);
	double circ_radius=-1;
	vector< cv::Point > cont_points;

	//structuring element
	int structE_size = 7;

	// PARAMETERS TO SET marc_vaio --- big blue ball
	//	double low_sat=50, high_sat =150;
	//	double low_hue=110, high_hue = 135;
	
	// Debug
	// 0 - No debug
	// 1 - Show windows
	// 2 - Show windows + set threshold parameters 

	// parameters to set C310 webcam---big blue ball
	/*double low_hue = 20, high_hue = 50; // 20 -50	
	double low_sat = 100, high_sat = 180;  
	*/
	

	// Hough circle parameters
	double dp =4, min_dist; //= src_gray.rows/8;
	double param_1 = 255, param_2 = 50;
	double min_radius = 5, max_radius = 1000;
	
	//ROI factor w.r.t Hough circle Radius
	int roi_fact = 3;

	
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	
	try
	{
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	
	cv::Mat imWebcam,imHSV, mask;
	cv::GaussianBlur(cv_ptr->image, imWebcam,cv::Size(11,11),11);
	cv::cvtColor( imWebcam, imHSV, CV_BGR2HSV );
	min_dist = imWebcam.rows/10;
	
	if ( debug == 2){
	    ROS_INFO("Segmentation: enter low and high hue thresholds [0 - 255] : ");
	    cin>>low_hue;
	    cin>>high_hue;
	    ROS_INFO("Segmentation: enter low and high saturation thresholds [0 255]");
	    cin>>low_sat;
	    cin>>high_sat;
	}
	cv::inRange(imHSV,cv::Scalar(low_hue,low_sat,0),cv::Scalar(high_hue,high_sat,255),mask);
	

	// FILL BLOBS
	vector<vector<cv::Point> > contours;
	cv::findContours(mask.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cv::Mat mask2 = cv::Mat::zeros(mask.size(), mask.type());
	cv::drawContours(mask2, contours, -1, cv::Scalar::all(255), CV_FILLED);
	
	// ERODE + DILATE
	cv::Mat str_elem = getStructuringElement(cv::MORPH_ELLIPSE,
                                       cv::Size( 2*structE_size + 1, 2*structE_size+1 ),
                                       cv::Point( structE_size, structE_size ) );

	cv::erode(mask2, mask2,str_elem);
	cv::dilate(mask2,mask2,str_elem);

	
	
	// obtain edges
	cv::Mat canny_im;
	cv::Canny(mask2, canny_im, 3, 100);

	// Circles detection
    	vector<cv::Vec3f> circles;
    	HoughCircles(canny_im, circles, CV_HOUGH_GRADIENT,
                 dp, min_dist, param_1, param_2,min_radius, max_radius );

	
	cv::Rect roi; 
	cv::Point offset, offset2;
	if(circles.size()!=0){ // the first circle is the most voted
		cv::Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
         	int radius = cvRound(circles[0][2]);
         	cv::circle( imWebcam, center, radius, cv::Scalar(0,0,255), 1, 1, 0 );
		
		// Create ROI near detected hough Circle
		offset = cv::Point(center.x - radius*roi_fact, center.y - radius*roi_fact);
		offset2 = cv::Point (radius*roi_fact*2, radius*roi_fact*2); 		

		//Check Roi boundaries
		if( offset.x <0 || offset.x >= mask2.cols){ 
			offset.x = 0;		
		}
		if(offset.y <0 || offset.y >=mask2.rows){
			offset.y = 0;		
		}
		
		if (offset.x + offset2.x >=mask2.cols){
			offset2.x = mask2.cols - offset.x-1 ;
		}
		if (offset.y + offset2.y >=mask2.rows){
			offset2.y = mask2.rows - offset.y-1 ;
		}

		// Create and display the region of interest
		cv::rectangle(imWebcam, offset , cv::Point( offset.x + offset2.x, offset.y + offset2.y), cv::Scalar(0,255,0));
		roi = cv::Rect(offset.x, offset.y, offset2.x, offset2.y);
		prev_roi = roi;	
		Never_detected=false;
	
	}else{ // When no circles are detected
		ROS_INFO("HOUGH CIRCLE: No circle detected yet");
		roi = prev_roi;
	}
	

	if (!Never_detected){	
		
		cv::Mat im_roi = mask2(roi);
		im_roi = im_roi.clone();
		
		if (debug == 1 || debug == 2){
		    cv::namedWindow( "Region of Interest",  CV_WINDOW_NORMAL| CV_GUI_EXPANDED  );
	        cv::imshow("Region of Interest",im_roi); 
		}
		
		// Obtain the contour of the blob in ROI
		cv::findContours(im_roi,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE,offset); 
		
		int ind=0, max_size=0;
		if( contours.size()!=0){	// from the biggest contour--> take some 5 points + radius + centroid	
			for (int i=0;i<contours.size();i++){
				if(contours[i].size()>= max_size){
					max_size = contours[i].size();
					ind = i;
				}
			}		
			// get the moments of the blob
			cv::Moments mu;
			mu = cv::moments( contours[ind], false );

			///  Get the mass center of the blob 
			cv::Point2f mc;
			centroid = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
			
			// get the radius from the blob area
			circ_radius = sqrt(cv::contourArea(contours[ind])/(atan(1)*4));
	
			// draw blue circle on ball contour
			cv::circle( imWebcam, centroid, circ_radius, cv::Scalar(255,0,0), 1, 1, 0 );

			int div = (int) contours[ind].size()/5;
			int i=0;
			for (int c=0;c<5;c++){
				cont_points.push_back( contours[ind][i]);
				cv::circle( imWebcam, contours[ind][i], 2, cv::Scalar(255,255,255), 1, 1, 0 );
				i = i+div;				
			}
			
		}
	}
	
	// Create output strings
	// message 1 - ball centroid + radius
	char str1[1024], str2[1024];
	sprintf( str1, "%d %d %.3f ", centroid.x, centroid.y, circ_radius );	
	string message1 = string(str1);

    // message 2 - Contour points
	string message2 = "";
	if (cont_points.size()!=0 ){
		for (int i=0; i<5;i++){
			sprintf(str2, "%d %d ", cont_points[i].x, cont_points[i].y );
			message2.append( str2);
		}
	}
		
	
	// Display images 
	if ( debug==1 || debug == 2){	
	    cv::namedWindow( "Segmentation",  CV_WINDOW_NORMAL| CV_GUI_EXPANDED  );
	    cv::namedWindow( "Filtered segmentation",  CV_WINDOW_NORMAL| CV_GUI_EXPANDED  );
	    cv::namedWindow( "Original image",  CV_WINDOW_NORMAL| CV_GUI_EXPANDED  );	
	       
	    cv::imshow("Segmentation",mask);
	    cv::imshow("Filtered segmentation",mask2);
	    cv::imshow("Original image",imWebcam);
	}
	
	// Publish message1 and message 2	
	std_msgs::String msg1, msg2;
    msg1.data = message1;
    msg1.data = message2;
    	
	pub1.publish(msg1);
	pub2.publish(msg2);
	
    cv::waitKey(3);
}




int main(int argc, char **argv)
{
	// Init ROS node
    ros::init(argc, argv, "ball_detector");
    ros::NodeHandle nh;	
	pub1 = nh.advertise<std_msgs::String>("/detection/message1", 2);
    pub2 = nh.advertise<std_msgs::String>("/detection/message2", 2);
    
    // Obtain the parameters from server
    nh.getParam("/detection/low_hue",  low_hue);
    nh.getParam("/detection/high_hue", high_hue);
    nh.getParam("/detection/low_sat",  low_sat);
    nh.getParam("/detection/high_sat", high_sat);
    nh.getParam("/detection/debug",    debug);
    ROS_INFO("Node: ball_detector initialized");
    
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);
	
	//OpenCV HighGUI call to create a display window on start-up.
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
	
    //OpenCV HighGUI call to destroy a display window on shut-down.
	//pub = it.advertise("/detection/image_hue", 1);
	
    // Continue execution forever
    ros::spin();
}
