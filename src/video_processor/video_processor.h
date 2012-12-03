
#ifndef __UAV_TARGET_TRACKING_VIDEO_PROCESSOR_ROSCPP_OPENCV__
#define __UAV_TARGET_TRACKING_VIDEO_PROCESSOR_ROSCPP_OPENCV__


#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define RAWVID              "Raw Video"
#define PROCVID             "Processed Video"

#define TRACK_COLOR         0
#define TRACK_PATTERN       1
#define TRACK_ARDRONETARGET 2

#define roiPadding          75

void onMouse(int, int, int, int, void *);

class VideoProcessor {

    private: 

        std::string inputtopic_name_;
        uint8_t mode_;

        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber videofeed_subscriber_;

        bool isDragging_, isInputDesired_, isRoiSelected_;
        cv::Point2i dragStart_, drag0_, drag1_;
        cv::Point minLoc_;
        cv::Rect roiRect_;

        cv::Scalar thresholdColor_, targetColorLow_, targetColorHigh_;
        
        cv::Mat kernel_, roi_, originalRoi_;

        void patternSelector(const sensor_msgs::ImageConstPtr& msg);
        void colorSelector(const sensor_msgs::ImageConstPtr& msg);
        void ardroneTargetSelector(const sensor_msgs::ImageConstPtr& msg);


    public:

        VideoProcessor(const std::string inputtopic_name_init, uint8_t mode);
        ~VideoProcessor();

        void processMouseHover(int &event, cv::Point2i hoverPoint);

};

void onMouse(int event, int x, int y, int, void *params);

extern VideoProcessor *p_videoProcessor;


#endif
