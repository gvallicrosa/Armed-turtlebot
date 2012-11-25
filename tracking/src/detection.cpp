//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
using namespace std;

// Global variables
int low_hue, high_hue, low_sat, high_sat, debug;
cv::Rect prev_roi;
bool Never_detected = true;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	cv::Point centroid = cv::Point(-1,-1);
	double circ_radius=-1;
	vector< cv::Point > cont_points;

	//structuring element
	int structE_size = 7;

	// PARAMETERS TO SET marc_vaio --- big blue ball
	/*	double low_sat=50, high_sat =150;
		double low_hue=110, high_hue = 135;
	*/

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
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
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
	
	// MANUAL INPUT OF THE THRESHOLDS
	/*cout<< "enter low and high hue"<<endl;
	cin>>low_hue;
	cin>>high_hue;
	cout<< "enter low and high saturation"<<endl;
	cin>>low_sat;
	cin>>high_sat;
	cout<< "-------------"<<endl<<endl;
	*/
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
         	// draw the circle outline
         	cv::circle( imWebcam, center, radius, cv::Scalar(0,0,255), 1, 1, 0 );
		
		// Create ROI near detected hough Circle
		offset = cv::Point(center.x - radius*roi_fact, center.y - radius*roi_fact);
		offset2 = cv::Point (radius*roi_fact*2, radius*roi_fact*2); 		

		//Check Roi boundaries
		//cout << "Image size = " << mask2.rows << " , " << mask2.cols <<endl;
		//cout<< " BEFORE BOUNDARIES top right : "<< offset.x << " , " <<offset.y <<endl;
		//cout << "bottom left : " << offset2.x + offset.x << " , " << offset2.y + offset.y <<endl<<endl;
		

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
		cout<< " HOUGH DIDNT DETECT ANY CIRCLE -->> PREVIOUS ROI USED"<<endl;
		roi = prev_roi;
	}
	

	if (!Never_detected){	
		
		cv::Mat im_roi = mask2(roi);
		im_roi = im_roi.clone();
		cv::namedWindow( "ROI",  CV_WINDOW_NORMAL| CV_GUI_EXPANDED  );
		cv::imshow("ROI",im_roi);
		
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
	
	//cv::imshow("WIND_MASK",mask);
	char str1[1024], str2[1024];
	sprintf( str1, "%d %d %.3f ", centroid.x, centroid.y, circ_radius );
	
	string message1 = string(str1);
	cout<< message1<<endl;

	string message2 = "";

	if (cont_points.size()!=0 ){
		for (int i=0; i<5;i++){
			sprintf(str2, "%d %d ", cont_points[i].x, cont_points[i].y );
			message2.append( str2);
		}
	}
		
	cout << message2<<endl;
	
	// DISPLAY IMAGES
	cv::namedWindow( "New_mask",  CV_WINDOW_NORMAL| CV_GUI_EXPANDED  );
	cv::namedWindow( "Filled_mask",  CV_WINDOW_NORMAL| CV_GUI_EXPANDED  );
	cv::namedWindow( "Original",  CV_WINDOW_NORMAL| CV_GUI_EXPANDED  );
	
	cv::imshow("Original",imWebcam);
	cv::imshow("New_mask",mask);
	cv::imshow("Filled_mask",mask2);

	cv::waitKey(3);
	
	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor in main().
	*/
	//Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
	cv_ptr->image = imWebcam;
        pub.publish(cv_ptr->toImageMsg());
	//prev = vec_hsv[2];
}




int main(int argc, char **argv)
{
	// Init ROS node
    ros::init(argc, argv, "ball_detector");
	ros::NodeHandle nh;
    
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
	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
	
    //OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW);
	pub = it.advertise("camera/image_processed", 1);
    // Continue execution forever
    ros::spin();
}
