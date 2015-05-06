/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ranger_librarian_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ranger_librarian_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
    // delete allocated objects
    delete  nh_;
    delete it_;
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"ranger_librarian_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    nh_ = new ros::NodeHandle("");
    it_ = new image_transport::ImageTransport( *nh_);


	// Add your ros communications here.
    chatter_publisher = nh_->advertise<std_msgs::String>("chatter", 1000);
    // Subscribers
    sub_rgb_    = it_->subscribe("/usb_cam/image_raw", 1, &QNode::rgb_callback, this);

	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(30);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


void QNode::rgb_callback(const sensor_msgs::ImageConstPtr &msg)
{

    std::cout << "callback" << std::endl;
    try
    {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        conversion_mat_ = cv_ptr->image.clone();

        // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation

        Q_EMIT rgbImageUpdated();
    }
    catch (cv_bridge::Exception& e)
    {
        printf("cv_bridge exception: %s", e.what());
        return;
    }


}





}  // namespace ranger_librarian_gui
