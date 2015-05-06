/**
 * @file /include/ranger_librarian_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ranger_librarian_gui_QNODE_HPP_
#define ranger_librarian_gui_QNODE_HPP_

/****************************************************************************/
// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>

// ROS messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

//CV bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Other
#include <string>
#include <QThread>
#include <QStringListModel>

// node rate
static const int NODE_RATE = 31;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ranger_librarian_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void rgbImageUpdated();

private:
    int init_argc;
    char** init_argv;
    ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    // Node
    ros::NodeHandle* nh_;
    image_transport::ImageTransport* it_;

    // Node loop rate
//    ros::Rate nh_rate_;

    // Publishers
    ros::Publisher pub_user_rgb_;

    // Subscribers
    image_transport::Subscriber sub_rgb_;

    ros::Subscriber sub_depth_low_duration_;
    ros::Subscriber sub_depth_low_action_;
    ros::Subscriber sub_scale_;
    ros::Subscriber sub_scale_filtered_;




public:
    // Callbacks
    void rgb_callback(const sensor_msgs::ImageConstPtr& msg);

    cv::Mat conversion_mat_;
    QImage qimage_;

};

}  // namespace ranger_librarian_gui

#endif /* ranger_librarian_gui_QNODE_HPP_ */
