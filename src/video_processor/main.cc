#include "ros/ros.h"

#include "video_processor.h"


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "video_processor");
    p_videoProcessor = new VideoProcessor("/ardrone/front/image_raw", TRACK_PATTERN);

    ros::spin();
}
