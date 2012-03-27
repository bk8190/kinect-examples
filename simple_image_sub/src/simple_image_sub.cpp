// Author: Bill Kulp
// Simple program that subscribes to and publishes an image
// 3/26/2012

// The following cheat sheet is a nice OpenCV reference
// http://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&ved=0CCgQFjAA&url=http%3A%2F%2Fwear-ar-fhbingen.googlecode.com%2Ffiles%2Fopencv_cheatsheet.pdf&ei=jx1xT-6fO5GL0QG2v_TRAQ&usg=AFQjCNGcF5Vy3A2rk_VHR_Vw1d-EWuEKGQ&sig2=Qo7gbfcpZ9PJa3ROAdh-_w

#include <ros/ros.h>
#include <boost/format.hpp>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

using std::string;

// Global variables here
image_transport::Publisher image_pub_;
string window_name_;

// A callback for an image
void imageCB(const sensor_msgs::ImageConstPtr& image_msg)
{
	// Convert the image from ROS format to OpenCV format
	// cv_ptr is a CvImagePtr which has a cv::Mat as a class member.
	// See http://www.ros.org/doc/api/cv_bridge_redesign/html/namespacecv__bridge.html
	cv_bridge::CvImagePtr cv_ptr;
	try	{
		cv_ptr = cv_bridge::toCvCopy(image_msg);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		return;
	}
	
	ROS_INFO_STREAM(boost::format("Callback got an image in format %s, size %dx%d")
		% cv_ptr->encoding % cv_ptr->image.rows % cv_ptr->image.cols );

	// Draw something on the image
	if( cv_ptr->image.rows > 100 && cv_ptr->image.cols > 100 )
	{
		cv::circle(cv_ptr->image, cv::Point(50, 50), 40, CV_RGB(255,0,0));
	}

	// Display the image.  The window does not update without the cvWaitKey.
	cv::imshow(window_name_.c_str(), cv_ptr->image);
	cvWaitKey(5);
	
	// Publish the updated image
  image_pub_.publish(cv_ptr->toImageMsg()); 
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "simple_image_sub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	// Get some parameters
	ros::NodeHandle private_nh("~");
	string mystringparam;
	double mydoubleparam;
	private_nh.param("test" , mystringparam, string("this is a default value"));
	private_nh.param("test2", mydoubleparam, 1.0 );
	ROS_INFO_STREAM("mystringparam = " << mystringparam);
	ROS_INFO_STREAM("mydoubleparam = " << mydoubleparam);
	
	// Subscribe to an image. Note the use of image_transport::Subscriber instead of ros::Subscriber
	image_transport::Subscriber image_sub = it.subscribe("in_image", 1, boost::bind(&imageCB, _1) );
	
	// publisher for the output image, again using image_transport
  image_pub_ = it.advertise(ros::this_node::getName() + "/out_image", 1);

	window_name_ = "Image from Kinect";
	cv::namedWindow(window_name_.c_str());
	ROS_INFO("Done initializing, going into spin mode.");

	while( ros::ok() )
	{
  	ros::spinOnce();
  }
  
	return 0;
}
