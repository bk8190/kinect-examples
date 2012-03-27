// Author: Bill Kulp
// Simple program that subscribes to and publishes a point cloud and image
// 3/26/2012

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <boost/format.hpp>

// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using std::string;

// Shorthand for our point cloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;


// Global variables here
ros::Publisher             cloud_pub_;
image_transport::Publisher image_pub_;
string window_name_;

// A magical callback that combines an image, cam info, and point cloud
void allCB(const sensor_msgs::ImageConstPtr& image_msg, 
           const PointCloudXYZRGB::ConstPtr& cloud_msg,
           const sensor_msgs::CameraInfo::ConstPtr& cam_msg)
{
	// Convert the image from ROS format to OpenCV format
	cv_bridge::CvImagePtr cv_ptr;
	try	{
		cv_ptr = cv_bridge::toCvCopy(image_msg);//, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	// cv_ptr now points to a CvImagePtr which has a cv::Mat as a class member.
	// To get it, use cv_ptr->image
	
	ROS_INFO_STREAM(boost::format("Callback got an image in format %s, size %dx%d")
		%cv_ptr->encoding %cv_ptr->image.size().width %cv_ptr->image.size().height);

	// Show the image.  For some reason, the window does not update without the cvWaitKey.
	cv::imshow(window_name_.c_str(), cv_ptr->image);
	cvWaitKey(1);
	
	// Just republish the data we got
  image_pub_.publish(image_msg);
	cloud_pub_.publish(cloud_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "rate_limiter");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	image_transport::ImageTransport it(nh);
	
	// Get some parameters
	string mystringparam;
	private_nh.param("test", mystringparam, string("this is a default value"));
	ROS_INFO_STREAM("mystringparam = " << mystringparam);
	
	double mydoubleparam;
	private_nh.param("test2", mydoubleparam, 1.0);
	ROS_INFO_STREAM("mydoubleparam = " << mydoubleparam);
	
	// Subscribe to an image, cloud, and camera info.
	// Note the use of image_transport::SubscriberFilter and message_filters::Subscriber.  These allow for synchronization of the topics.
	image_transport::SubscriberFilter                    image_sub   (it, "in_image", 1);
	message_filters::Subscriber<PointCloudXYZRGB>        cloud_sub   (nh, "in_cloud", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, "in_cam_info", 1);
	
	// This sync policy will invoke a callback if it receives one of each message with matching timestamps
	typedef message_filters::sync_policies::ExactTime
	   <sensor_msgs::Image, PointCloudXYZRGB, sensor_msgs::CameraInfo> MySyncPolicy;
	
	// Synchronize the three topics.  MySyncPolicy(10) tells it to maintain a buffer of 10 messages.
	message_filters::Synchronizer<MySyncPolicy>
	   sync(MySyncPolicy(10), image_sub, cloud_sub, cam_info_sub);
	
	// Hook the callback into the sync policy
	sync.registerCallback( boost::bind(&allCB, _1, _2, _3) );
	
	// publisher for the image
	string image_topic = nh.resolveName("out_image");
  image_pub_ = it.advertise(image_topic, 1);

	// publisher for the point cloud
	string cloud_topic = nh.resolveName("out_cloud");
	cloud_pub_ = nh.advertise<PointCloudXYZRGB>(cloud_topic, 1);

	window_name_ = "Image from Kinect";
	cv::namedWindow(window_name_.c_str());
	ROS_INFO("Done initializing, going into spin mode.");

	while( ros::ok() )
	{
  	ros::spinOnce();
  }
  
	return 0;
}
