#include <algorithm>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Empty.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "video_processor.h"


namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;

VideoProcessor *p_videoProcessor;


void VideoProcessor::videoframeCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
    Mat displayImg, procImg; 

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv bridge exception: %s", e.what());
        return;
    }

    switch (mode_) {
        case TRACK_PATTERN:
            patternSelector(cv_ptr->image, displayImg, procImg);
            break;

        case TRACK_COLOR:
            colorSelector(cv_ptr->image, displayImg, procImg);
            break;

        default:
            displayImg = cv_ptr->image;
            procImg = cv_ptr->image;
            break;
    }

    imshow(RAWVID, displayImg);
    imshow(PROCVID, procImg);
    waitKey(20);
}

void VideoProcessor::patternSelector(Mat input, Mat displayImg, Mat procImg) {

    int roiOffsetX, roiOffsetY, roiWidth, roiHeight;
    double minVal, maxVal, originalMinVal;
    Point maxLoc, originalMinLoc, roiOffset;
    Size roiSize;
    Scalar targetColor, targetColorOffsets;
    Mat procImgRoi, matchFiltered;
    vector <vector <Point> > contours;

    GaussianBlur(input, displayImg, Size(3, 3), 0);
    cvtColor(displayImg, procImg, CV_RGB2GRAY);

    if (isDragging_) {
        rectangle(displayImg, drag0_, drag1_, Scalar(255, 0, 0));
    }     
    if (isInputDesired_) {
        roi_ = Mat(procImg.clone(), Rect(drag0_.x, drag0_.y, drag1_.x - drag0_.x, drag1_.y - drag0_.y));
        roi_.locateROI(roiSize, minLoc_);
        originalRoi_ = roi_.clone();

        isInputDesired_ = false;
        isRoiSelected_ = true; }

    if (isRoiSelected_) {
        roiOffsetX = (minLoc_.x > roiPadding) ? minLoc_.x - roiPadding : 0;
        roiOffsetY = (minLoc_.y > roiPadding) ? minLoc_.y - roiPadding : 0;
        roiWidth = roi_.cols + roiPadding*2;
        if (roiOffsetX + roiWidth > procImg.cols)
            roiOffsetX = procImg.cols - roiWidth;
        roiHeight = roi_.rows + roiPadding*2;
        if (roiOffsetY + roiHeight > procImg.rows)
            roiOffsetY = procImg.rows - roiHeight;

        ROS_INFO("offsetx %d, offset y %d", roiOffsetX, roiOffsetY);

        procImgRoi = Mat(procImg, Rect(roiOffsetX, roiOffsetY, roiWidth, roiHeight));
        matchFiltered.create(procImgRoi.cols - roi_.cols + 1, procImgRoi.rows - roi_.rows + 1, CV_8U);

        matchTemplate(procImgRoi, roi_, matchFiltered, CV_TM_SQDIFF_NORMED);
        minMaxLoc(matchFiltered, &minVal, &maxVal, &minLoc_, &maxLoc);

        minLoc_.x += roiOffsetX;
        minLoc_.y += roiOffsetY;

        ROS_INFO("Min value: %f", minVal);
        if (minVal > 0.01) {
            matchTemplate(procImg, originalRoi_, matchFiltered, CV_TM_SQDIFF_NORMED);
            minMaxLoc(matchFiltered, &originalMinVal, &maxVal, &originalMinLoc, &maxLoc);
            if (originalMinVal < 0.006) {
                roi_ = originalRoi_.clone();
                minLoc_ = originalMinLoc;
                rectangle(displayImg, originalMinLoc, Point(originalMinLoc.x + roi_.cols, originalMinLoc.y + roi_.rows), Scalar(0, 255, 0), 1, 8, 0);
            }
            return;
        }
        else if (minVal > 0.005) {
            matchTemplate(procImg, originalRoi_, matchFiltered, CV_TM_SQDIFF_NORMED);
            minMaxLoc(matchFiltered, &originalMinVal, &maxVal, &originalMinLoc, &maxLoc);

            if (originalMinVal < 0.005) {
                roi_ = originalRoi_.clone();
                minLoc_ = originalMinLoc;
            }
            else {
                roi_ = Mat(procImg.clone(), Rect(minLoc_.x, minLoc_.y, roi_.cols, roi_.rows));
            }
            rectangle(displayImg, minLoc_, Point(minLoc_.x + roi_.cols, minLoc_.y + roi_.rows), Scalar(0, 255, 0), 1, 8, 0);
            return;
        }
        else {
            rectangle(displayImg, minLoc_, Point(minLoc_.x + roi_.cols, minLoc_.y + roi_.rows), Scalar(0, 255, 0), 1, 8, 0);
            return;
        }
    }
}

void VideoProcessor::colorSelector(Mat input, Mat displayImg, Mat procImg) {

    unsigned i, j;
    cv_bridge::CvImagePtr cv_ptr;
    Mat mask;
    Scalar targetColor, targetColorOffsets;
    vector <vector <Point> > contours;

    GaussianBlur(cv_ptr->image, displayImg, Size(3, 3), 0);
    cvtColor(displayImg, procImg, CV_RGB2HSV);

    if (isDragging_) {
        rectangle(displayImg, drag0_, drag1_, Scalar(255, 0, 0));
    }     
    if (isInputDesired_) {

        mask = Mat(Mat::zeros(procImg.size(), CV_8U), Rect(drag0_.x, drag0_.y, drag1_.x - drag0_.x, drag1_.y - drag0_.y));
        mask = 255;

        meanStdDev(procImg, targetColor, targetColorOffsets, mask);
        
        targetColorOffsets[0] *= 4.25;
        targetColorOffsets[1] *= 7.50;
        targetColorOffsets[2] *= 7.50;

        ROS_INFO("h: %f, s: %f, v: %f", targetColorOffsets[0], targetColorOffsets[1], targetColorOffsets[2]);

        targetColorLow_[0]   = targetColor[0] - targetColorOffsets[0];
        targetColorHigh_[0]  = targetColor[0] + targetColorOffsets[0];

        targetColorLow_[1]   = targetColor[1] - targetColorOffsets[1];
        targetColorHigh_[1]  = targetColor[1] + targetColorOffsets[1];

        targetColorLow_[2]   = targetColor[2] - targetColorOffsets[2];
        targetColorHigh_[2]  = targetColor[2] + targetColorOffsets[2];

        isInputDesired_ = false;
    }

    inRange(procImg, targetColorLow_, targetColorHigh_, procImg);

    erode(procImg, procImg, kernel_);
    dilate(procImg, procImg, kernel_);

    findContours(procImg, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_TC89_L1);

    // draw stuff
    polylines(displayImg, contours, true, Scalar(255, 0, 0), 1, 8, 0);
    for (i = 0; i < contours.size(); i++) {
        for (j = 0; j < contours[i].size(); j++) {
            circle(displayImg, contours[i][j], 5, Scalar(0, 0, 255));
        }
    }

    return;
}

VideoProcessor::VideoProcessor(const string inputtopic_name_init, uint8_t mode) 
    : it_(n_), 
    targetColorLow_(0, 0, 0), targetColorHigh_(0, 0, 0),
    kernel_(getStructuringElement(MORPH_CROSS, Size(3, 3)))
{
    inputtopic_name_ = inputtopic_name_init;
    videofeed_subscriber_ = it_.subscribe(inputtopic_name_, 1, &VideoProcessor::videoframeCallback, this);

    isInputDesired_ = false;
    isDragging_ = false;
    isRoiSelected_ = false;

    mode_ = mode;

    namedWindow(RAWVID);
    setMouseCallback(RAWVID, onMouse, 0);

    namedWindow(PROCVID);

}

VideoProcessor::~VideoProcessor() {
    destroyWindow(RAWVID);
    destroyWindow(PROCVID);
}

void VideoProcessor::processMouseHover(int &event, Point2i hoverPoint) {

    if (event == CV_EVENT_LBUTTONDOWN) {
        isDragging_ = true;
        dragStart_.x = hoverPoint.x;
        dragStart_.y = hoverPoint.y;
    }

    if (isDragging_) {
        if (event != CV_EVENT_LBUTTONUP) {
            drag0_.x = min(hoverPoint.x, dragStart_.x);
            drag0_.y = min(hoverPoint.y, dragStart_.y);

            drag1_.x = max(hoverPoint.x, dragStart_.x);
            drag1_.y = max(hoverPoint.y, dragStart_.y);
        }
        else {
            isDragging_ = false;
            isInputDesired_ = true;
        }
    }

}


void onMouse(int event, int x, int y, int, void *params) {

    p_videoProcessor->processMouseHover(event, Point2i(x, y));

}
