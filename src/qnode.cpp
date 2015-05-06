/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

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
    init_argv(argv),
    read_label_(false), read_label_success_(false), weight_max_reached_(false),
    lr_(OCR_FRAME_SKIP, QUEUE_MAX_LENGTH, QUEUE_ACCEPT_RATE)
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

    string rgb_image_topic;
    string depth_low_duration;
    string depth_low_action;
    string scale_topic;
    string scale_filtered_topic;

    // get parameters for subscribers, if not use defaults
    nh_->param<string>("rgb_image", rgb_image_topic, RGB_IMAGE_TOPIC);
    nh_->param<string>("depth_low_duration", depth_low_duration, DEPTH_LOW_DURATION_TOPIC);
    nh_->param<string>("depth_low_action", depth_low_action,     DEPTH_BELOW_TIMER_TOPIC);

    nh_->param<string>("scale", scale_topic, SCALE_TOPIC);
    nh_->param<string>("scale_filtered", scale_filtered_topic, SCALE_FILTERED_TOPIC);




	// Add your ros communications here.
    chatter_publisher = nh_->advertise<std_msgs::String>("chatter", 1000);
    // Subscribers
    sub_rgb_    = it_->subscribe("/usb_cam/image_raw", 1, &QNode::rgb_callback, this);

    sub_depth_low_duration_  = nh_->subscribe<const std_msgs::Float64&>(depth_low_duration, 1, &QNode::depth_low_duration_callback, this);
    sub_depth_low_action_  =   nh_->subscribe<const std_msgs::String&>(depth_low_action, 1, &QNode::depth_low_action_callback, this);

    sub_scale_  =           nh_->subscribe<const std_msgs::Float64&>(scale_topic, 1, &QNode::scale_callback, this);
    sub_scale_filtered_  =  nh_->subscribe<const ranger_librarian::WeightFiltered&>(scale_filtered_topic, 1, &QNode::scale_filtered_callback, this);

    // get the rest of paramters
    nh_->param("weight_max_allowed", weight_max_allowed_,        double(5));
    nh_->param("time_depth_low_read", time_depth_low_read_,      double(1.5));
    nh_->param("time_wait_read_label", time_wait_read_label_,    double(6));
    nh_->param("time_wait_add_book", time_wait_add_book_,        double(8));

    ROS_INFO("qnode init finished");

    start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(31);
	int count = 0;

	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
//    	log(Info,std::string("I sent: ")+msg.data);

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
    if (DEBUG) {
        printf("rgb_callback msg received.\n");
    }

    try
    {
        // First let cv_bridge do its magic
        cv_ptr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);


        // process frame with orc if needed
        read_label_success_ = lr_.processFrame(cv_ptr_->image);

        // image must be copied since it uses the cv_ptr->image for storage which is asynchronously overwritten in the next callback invocation
        user_image_ = cv_ptr_->image.clone();

        // prepare user output image view
        lr_.prepareUserImage(user_image_);

        Q_EMIT rgbImageUpdated();
    }
    catch (cv_bridge::Exception& e)
    {
        printf("cv_bridge exception: %s", e.what());
        return;
    }


}


void QNode::depth_low_duration_callback(const std_msgs::Float64& msg) {
    // UNUSED
//    if (DEBUG) {
//        printf("depth_below_duration msg received:  %0.2f\n", msg.data);
//    }

//    double depth_below_duration = msg.data;

//    if (depth_below_duration > time_depth_low_read_) {
//        read_label_ = true;
//    } else {
//        read_label_ = false;
//    }
}

void QNode::depth_low_action_callback(const std_msgs::String& msg) {

    string depth_low_action = msg.data;

    if (DEBUG) {
        printf("depth_low_action msg received:  %s\n", depth_low_action.c_str());
    }

    if (depth_low_action.compare("stop")!=0 ) {
        read_label_ = false;
    } else {
        read_label_ = true;
    }

    if (read_label_) {
        std::cout <<  "trying to read label " << std::endl;
        //wait for book label
//        if (book_read_label()) {

//            std::cout <<  "read label success! waiting for book " << std::endl;

//            if (book_read_weight()) {
//                std::cout <<  "book add success! " << std::endl;
//            } else {
//                std::cout <<  "book add failed! " << std::endl;
//            }

//        } else {
//             std::cout <<  "read label failed, timeout " << std::endl;

//        }
        read_label_ = false;
        std::cout << "move around" << std::endl;
    }
}

void QNode::scale_callback(const std_msgs::Float64& msg) {

//    double weight_current = msg.data;

//    if (DEBUG) {
//        printf("Scale msg received:  %0.2f\n", weight_current);
//    }

//    if (weight_current > weight_max_allowed_) {
//        weight_max_reached_ = true;
//    } else {
//        weight_max_reached_ = false;
//    }
//    // implement control of max weight reached
}

void QNode::scale_filtered_callback(const ranger_librarian::WeightFiltered& msg) {

    double weight_stable = msg.weight_stable;
    double weight_change = msg.weight_change;
    bool weight_changed =  msg.weight_changed;

    if (DEBUG) {
        printf("scale_filtered_callback msg received:  %0.3f\n", weight_stable);
    }

//    if (weight_changed) {
//        if (weight_change > 0) {
//            last_book_add_time_ = msg.change_time;
//            std::cout << "book added" << std::endl;
//        } else {
//            std::cout << "book removed" << std::endl;
//        }
//    } else {
//        // no change
//    }

}




}  // namespace ranger_librarian_gui
