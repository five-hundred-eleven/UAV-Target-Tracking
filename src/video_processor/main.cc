#include "ros/ros.h"

#include "video_processor.h"


int main(int argc, char *argv[]) {

    std::string videofeedTopic = "/ardrone/front/image_raw";
    std::string defaultStr = "default", color = "color", pattern = "pattern", ardroneTarget = "ardronetarget";
    uint8_t trackMode = TRACK_PATTERN;

    ros::init(argc, argv, "video_processor");

    
    if (argc >= 2) {
        if (argv[1] != defaultStr) {
            videofeedTopic = argv[1];
        }
    }
    if (argc >= 3) {
        if (argv[2] == color)
            trackMode = TRACK_COLOR;
        else if (argv[2] == pattern)
            trackMode = TRACK_PATTERN;
        else if (argv[2] == ardroneTarget)
            trackMode = TRACK_ARDRONETARGET;
    }


    p_videoProcessor = new VideoProcessor(videofeedTopic, trackMode);

    ros::spin();
}
