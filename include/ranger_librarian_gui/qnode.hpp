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
#include <ros/network.h>
#include <ros/callback_queue.h>

// ROS messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <ranger_librarian/WeightFiltered.h>

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

#include "label_reader.hpp"

using std::string;

/**@brief Flag for printing debug messages.*/
bool const DEBUG = true;

/**@brief Default rosparam server values.*/
static const std::string RGB_IMAGE_TOPIC = "/usb_cam/image_raw";
static const string SCALE_TOPIC = "/scale";
static const string SCALE_FILTERED_TOPIC = "/scale_filtered";
static const string DEPTH_LOW_DURATION_TOPIC = "/depth_below_timer/depth_low_duration";
static const string DEPTH_BELOW_TIMER_TOPIC = "/depth_below_timer/depth_low_action";

// node rate
static const int NODE_RATE = 31;

/**@brief Default OCR parameters.*/
static const int OCR_FRAME_SKIP = 5;        // parameter to process each xx frame with OCR
static const int QUEUE_MAX_LENGTH = 10;     // how many historical values to keep in queue
static const double QUEUE_ACCEPT_RATE = 0.7;// last repeated element acceptance rate



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

    //  scale callback
    double weight_max_allowed_;             // maximum weight allowed

    // read label parameters
    double time_depth_low_read_;            // time (s) for depth val = 0 to start label read
    double time_wait_read_label_;           // time (s) to wait for reading a book label
    double time_wait_add_book_;             // time (s) to wait for adding a book

    // Label reader object
    LabelReader lr_;

    // pointer to obtained cv image
    cv_bridge::CvImageConstPtr cv_ptr_;

    // control boolean variables
    bool read_label_;
    bool read_label_success_;
    bool weight_max_reached_;


public:
    // Callbacks
    void rgb_callback(const sensor_msgs::ImageConstPtr& msg);

    void depth_low_duration_callback(const std_msgs::Float64& msg);
    void depth_low_action_callback(const std_msgs::String& msg);
    void scale_callback(const std_msgs::Float64& msg);
    void scale_filtered_callback(const ranger_librarian::WeightFiltered& msg);

    cv::Mat user_image_;

};

}  // namespace ranger_librarian_gui

#endif /* ranger_librarian_gui_QNODE_HPP_ */
