#include <algorithm>
#include <cmath>

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


VideoProcessor::VideoProcessor(const string inputtopic_name_init, uint8_t mode) 
    : it_(n_), 
    thresholdColor_(0), targetColorLow_(0, 0, 0), targetColorHigh_(0, 0, 0),
    kernel_(getStructuringElement(MORPH_ELLIPSE, Size(3, 3)))
{
    inputtopic_name_ = inputtopic_name_init;

    isInputDesired_ = false;
    isDragging_ = false;
    isRoiSelected_ = false;

    mode_ = mode;
    switch (mode_) {
        case TRACK_PATTERN:
            videofeed_subscriber_ = it_.subscribe(inputtopic_name_, 1, &VideoProcessor::patternSelector, this);
            ROS_INFO("Mode: track pattern");
            break;
        case TRACK_COLOR:
            videofeed_subscriber_ = it_.subscribe(inputtopic_name_, 1, &VideoProcessor::colorSelector, this);
            ROS_INFO("Mode: track color");
            break;
        case TRACK_ARDRONETARGET:
            videofeed_subscriber_ = it_.subscribe(inputtopic_name_, 1, &VideoProcessor::ardroneTargetSelector, this);
            ROS_INFO("Mode: track ardrone target");
            break;

        default:
            return;
    }

    namedWindow(RAWVID);
    setMouseCallback(RAWVID, onMouse, 0);

    namedWindow(PROCVID);

}

VideoProcessor::~VideoProcessor() {
    destroyWindow(RAWVID);
    destroyWindow(PROCVID);
}


void VideoProcessor::patternSelector(const sensor_msgs::ImageConstPtr& msg) {

    int roiOffsetX, roiOffsetY, roiWidth, roiHeight;
    double minVal, maxVal, originalMinVal;
    Point maxLoc, originalMinLoc, roiOffset;
    Size roiSize;
    Scalar targetColor, targetColorOffsets;
    Mat bwImgRoi, bwImg, displayImg, procImg;
    vector <vector <Point> > contours;
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv bridge exception: %s", e.what());
        return;
    }

    GaussianBlur(cv_ptr->image, displayImg, Size(3, 3), 0);
    cvtColor(displayImg, bwImg, CV_RGB2GRAY);

    if (isDragging_) {
        rectangle(displayImg, drag0_, drag1_, Scalar(255, 0, 0));
    }     
    if (isInputDesired_) {
        roi_ = Mat(bwImg.clone(), Rect(drag0_.x, drag0_.y, drag1_.x - drag0_.x, drag1_.y - drag0_.y));
        roi_.locateROI(roiSize, minLoc_);
        originalRoi_ = roi_.clone();

        isInputDesired_ = false;
        isRoiSelected_ = true; 
    }

    if (isRoiSelected_) {
        roiOffsetX = (minLoc_.x > roiPadding) ? minLoc_.x - roiPadding : 0;
        roiOffsetY = (minLoc_.y > roiPadding) ? minLoc_.y - roiPadding : 0;
        roiWidth = roi_.cols + roiPadding*2;
        if (roiOffsetX + roiWidth > bwImg.cols)
            roiOffsetX = bwImg.cols - roiWidth;
        roiHeight = roi_.rows + roiPadding*2;
        if (roiOffsetY + roiHeight > bwImg.rows)
            roiOffsetY = bwImg.rows - roiHeight;


        bwImgRoi = Mat(bwImg, Rect(roiOffsetX, roiOffsetY, roiWidth, roiHeight));
        procImg.create(bwImgRoi.cols - roi_.cols + 1, bwImgRoi.rows - roi_.rows + 1, CV_8U);

        matchTemplate(bwImgRoi, roi_, procImg, CV_TM_SQDIFF_NORMED);
        minMaxLoc(procImg, &minVal, &maxVal, &minLoc_, &maxLoc);

        minLoc_.x += roiOffsetX;
        minLoc_.y += roiOffsetY;

        ROS_INFO("Min value: %f", minVal);
        if (minVal > 0.02) {
            matchTemplate(bwImg, originalRoi_, procImg, CV_TM_SQDIFF_NORMED);
            minMaxLoc(procImg, &originalMinVal, &maxVal, &originalMinLoc, &maxLoc);
            if (originalMinVal < 0.012) {
                roi_ = originalRoi_.clone();
                minLoc_ = originalMinLoc;
                rectangle(displayImg, originalMinLoc, Point(originalMinLoc.x + roi_.cols, originalMinLoc.y + roi_.rows), Scalar(0, 255, 0), 1, 8, 0);
            }
        }
        else if (minVal > 0.010) {
            matchTemplate(bwImg, originalRoi_, procImg, CV_TM_SQDIFF_NORMED);
            minMaxLoc(procImg, &originalMinVal, &maxVal, &originalMinLoc, &maxLoc);

            if (originalMinVal < 0.010) {
                roi_ = originalRoi_.clone();
                minLoc_ = originalMinLoc;
            }
            else {
                roi_ = Mat(bwImg.clone(), Rect(minLoc_.x, minLoc_.y, roi_.cols, roi_.rows));
            }
            rectangle(displayImg, minLoc_, Point(minLoc_.x + roi_.cols, minLoc_.y + roi_.rows), Scalar(0, 255, 0), 1, 8, 0);
        }
        else {
            rectangle(displayImg, minLoc_, Point(minLoc_.x + roi_.cols, minLoc_.y + roi_.rows), Scalar(0, 255, 0), 1, 8, 0);
        }
    }

    if (!displayImg.empty()) {
        imshow(RAWVID, displayImg);
    }
    if (!procImg.empty()) {
        imshow(PROCVID, procImg);
    }
    waitKey(20);

}

void VideoProcessor::colorSelector(const sensor_msgs::ImageConstPtr& msg) {

    unsigned i, j;
    Mat mask, roi, displayImg, procImg;
    Scalar targetColor, targetColorOffsets;
    vector <vector <Point> > contours;
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv bridge exception: %s", e.what());
        return;
    }


    GaussianBlur(cv_ptr->image, displayImg, Size(3, 3), 0);
    cvtColor(displayImg, procImg, CV_RGB2HSV);

    if (isDragging_) {
        rectangle(displayImg, drag0_, drag1_, Scalar(255, 0, 0));
    }
    if (isInputDesired_) {

        mask = Mat::zeros(procImg.size(), CV_8U);
        roi = Mat(mask, Rect(drag0_.x, drag0_.y, drag1_.x - drag0_.x, drag1_.y - drag0_.y));
        roi = 255;

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

    dilate(procImg, procImg, kernel_);
    erode(procImg, procImg, kernel_);

    findContours(procImg.clone(), contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_TC89_L1);

    // draw stuff
    polylines(displayImg, contours, true, Scalar(255, 0, 0), 1, 8, 0);
    /*
    for (i = 0; i < contours.size(); i++) {
        for (j = 0; j < contours[i].size(); j++) {
            circle(displayImg, contours[i][j], 5, Scalar(0, 0, 255));
        }
    }
    */

    if (!displayImg.empty()) {
        imshow(RAWVID, displayImg);
    }
    if (!procImg.empty()) {
        imshow(PROCVID, procImg);
    }
    waitKey(20);

}

void VideoProcessor::ardroneTargetSelector(const sensor_msgs::ImageConstPtr& msg) {

    unsigned i, j, k;
    Mat mask, roi, displayImg, procImg;
    Scalar targetColor, targetColorOffsets;
    vector <vector <Point> > contours;
    vector <Vec3f> circles;
    cv_bridge::CvImagePtr cv_ptr;
    vector <RotatedRect> rotatedRects;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv bridge exception: %s", e.what());
        return;
    }


    GaussianBlur(cv_ptr->image, displayImg, Size(3, 3), 0);
    cvtColor(displayImg, procImg, CV_RGB2GRAY);

    if (isDragging_) {
        rectangle(displayImg, drag0_, drag1_, Scalar(255, 0, 0));
    }
    if (isInputDesired_) {

        roiRect_ = Rect(drag0_.x, drag0_.y, drag1_.x - drag0_.x, drag1_.y - drag0_.y);
        
        ROS_INFO("v: %f", thresholdColor_[0]);

        isInputDesired_ = false;
        isRoiSelected_ = true;
    }

    if (isRoiSelected_) {

        roi = Mat(procImg, roiRect_);
        adaptiveThreshold(roi, roi, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 35, 0);
        //threshold(roi, roi, 255, 


        // erode() shrinks dark areas
        erode(roi, roi, kernel_);
        erode(roi, roi, kernel_);

        // dilate() expands dark areas 
        dilate(roi, roi, kernel_);
        dilate(roi, roi, kernel_);

        //HoughCircles(roi, circles, CV_HOUGH_GRADIENT, 2, 50, 100, 70, 20, 150); 
        findContours(roi.clone(), contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

        RotatedRect nextEllipse, nextRect;
        double nextEllipseArea, nextRectArea, nextContourArea;

        for (i = 0; i < contours.size(); i++) {
            for (j = 0; j < contours[i].size(); j++) {
                contours[i][j].x += drag0_.x;
                contours[i][j].y += drag0_.y;
            }

            nextContourArea = contourArea(contours[i]);

            if (nextContourArea < 100)
                continue;

            if (contours[i].size() >= 5) {
                nextEllipse = fitEllipse(contours[i]);
                nextEllipseArea = CV_PI*nextEllipse.size.width*nextEllipse.size.height*0.25;
            }
            else {
                nextEllipseArea = 10000000;
            }

            if (contours[i].size() >= 4) {
                nextRect = minAreaRect(contours[i]);
                nextRectArea = nextRect.size.width*nextRect.size.height;
            }
            else {
                nextRectArea = 10000000;
            }

            if (nextEllipseArea < nextRectArea) {
                if (nextEllipseArea/nextContourArea > 1.20) 
                    continue;
                else
                    rotatedRects.push_back(nextEllipse); 
            }
            else {
                if (nextRectArea/nextContourArea > 1.20) 
                    continue;
                else
                    rotatedRects.push_back(nextRect); 
            }

        }

        if (rotatedRects.size() >= 2) {
            Size2f minSizeDiff = Size2f(100, 100), thisSizeDiff;
            int szThresh = 14, yThresh = 30, xThresh, widthDiff, heightDiff;
            float hwRatio1, hwRatio2;
            RotatedRect inner, outer;
            vector<Point2i> matches;
            for (i = 0; i < rotatedRects.size(); i++) {
                for (j = 0; j < i; j++) {

                    if (min(rotatedRects[i].size.width, rotatedRects[j].size.width) < 
                        abs(rotatedRects[i].center.x - rotatedRects[j].center.x))
                        continue;
                    if (min(rotatedRects[i].size.height, rotatedRects[j].size.height)/4 < 
                        abs(rotatedRects[i].center.y - rotatedRects[j].center.y))
                        continue;

                    hwRatio1 = rotatedRects[i].size.width / rotatedRects[i].size.height;
                    hwRatio2 = rotatedRects[j].size.width / rotatedRects[j].size.height;

                    if (hwRatio1 > hwRatio2) {
                        outer = rotatedRects[i];
                        inner = rotatedRects[j];
                    }
                    else {
                        outer = rotatedRects[j];
                        inner = rotatedRects[i];
                    }

                    inner.size.width *= 2;
                    if (inner.size.width > outer.size.width) {
                       continue; 
                    }

                    if (rotatedRects[i].center.x < rotatedRects[j].center.x) {
                        matches.push_back(Point(i, j));
                    }
                    else {
                        matches.push_back(Point(j, i));
                    }
                    ellipse(displayImg, inner, Scalar(0, 0, 255), 2);
                    ellipse(displayImg, outer, Scalar(0, 0, 255), 2);
                }
            }

            /*

            xThresh = 25;
            yThresh = 25;
            int upperRecti;
            RotatedRect ri1, ri2, rj1, rj2, upperRect1, upperRect2, lowerRect1, lowerRect2;
            vector<int> circleCandidatesFromImbedded, circleCandidatesfromLower;
            bool isCircleFound = false;

            for (i = 0; i < matches.size(); i++) {
                ri1 = rotatedRects[matches[i].x];
                ri2 = rotatedRects[matches[i].y];
                for (j = 0; j < i; j++) {
                    rj1 = rotatedRects[matches[j].x];
                    rj2 = rotatedRects[matches[j].y];

                    if ((min(ri1.size.width, rj1.size.width) < abs(ri1.center.x - rj1.center.x)) ||
                        (min(ri2.size.width, rj2.size.width) < abs(ri2.center.x - rj2.center.x)))
                        continue;

                    yThresh = ri1.size.height + rj1.size.height;
                    yThresh >>= 2;
                    if (abs(ri1.center.y - rj1.center.y) < yThresh) {
                        yThresh = ri2.size.height + rj2.size.height;
                        yThresh >>= 1;
                        if (abs(ri2.center.y - rj2.center.y) > yThresh)
                            continue;

                        if (ri1.size.width > rj1.size.width) {
                            upperRecti = i;

                            upperRect1 = ri1;
                            upperRect2 = ri2;
                            lowerRect1 = rj1;
                            lowerRect2 = rj2;
                        }
                        else {
                            upperRecti = j;

                            upperRect1 = rj1;
                            upperRect2 = rj2;
                            lowerRect1 = ri1;
                            lowerRect2 = ri2;
                        }

                        ROS_INFO("%f, %f", lowerRect1.angle, lowerRect2.angle);

                        if ((upperRect1.size.width < lowerRect1.size.width*2) || 
                            (upperRect2.size.width < lowerRect2.size.width*2))
                            continue;

                        if ((lowerRect1.size.width > lowerRect1.size.height) ||
                            (lowerRect2.size.width > lowerRect2.size.height))
                            continue;

                        circleCandidatesFromImbedded.push_back(upperRecti);
                        if (find(circleCandidatesfromLower.begin(), 
                                 circleCandidatesfromLower.end(), 
                                 upperRecti) 
                            != circleCandidatesfromLower.end()) {

                            isCircleFound = true;
                            break;

                        }

                        ellipse(displayImg, ri1, Scalar(255, 0, 0), 2);
                        ellipse(displayImg, ri2, Scalar(255, 0, 0), 2);

                        ellipse(displayImg, rj1, Scalar(255, 0, 0), 2);
                        ellipse(displayImg, rj2, Scalar(255, 0, 0), 2);

                    }
                    else {
                        if (ri1.center.y > rj1.center.y) {
                            upperRecti = i;

                            upperRect1 = ri1;
                            upperRect2 = ri2;
                            lowerRect1 = rj1;
                            lowerRect2 = rj2;
                        }
                        else {
                            upperRecti = j;

                            upperRect1 = rj1;
                            upperRect2 = rj2;
                            lowerRect1 = ri1;
                            lowerRect2 = ri2;
                        }

                        yThresh <<= 0;
                        if (abs(upperRect1.size.height - upperRect1.center.y + lowerRect1.center.y) > yThresh)
                            continue;
                        if (abs(upperRect2.size.height - upperRect2.center.y + lowerRect2.center.y) > yThresh)
                            continue;

                        if ((upperRect1.size.height < lowerRect1.size.height) || 
                             (upperRect2.size.height < lowerRect2.size.height))
                            continue;

                        xThresh = upperRect1.size.width + lowerRect1.size.width;
                        xThresh >> 1;
                        if (abs(upperRect1.size.width - lowerRect1.size.width) > xThresh || 
                            abs(upperRect2.size.width - lowerRect2.size.width) > xThresh)
                            continue;

                        circleCandidatesfromLower.push_back(upperRecti);
                        if (find(circleCandidatesFromImbedded.begin(), 
                                 circleCandidatesFromImbedded.end(), 
                                 upperRecti) 
                            != circleCandidatesFromImbedded.end()) {

                            isCircleFound = true;
                            break;

                        }

                        upperRect1.size.height += 4;
                        upperRect2.size.height += 4;
                        upperRect1.size.width  += 4;
                        upperRect2.size.width  += 4;

                        ellipse(displayImg, ri1, Scalar(0, 255, 0), 2);
                        ellipse(displayImg, ri2, Scalar(0, 255, 0), 2);

                        upperRect1.size.height -= 4;
                        upperRect2.size.height -= 4;
                        upperRect1.size.width  -= 4;
                        upperRect2.size.width  -= 4;

                        ellipse(displayImg, rj1, Scalar(0, 255, 0), 2);
                        ellipse(displayImg, rj2, Scalar(0, 255, 0), 2);


                    }

                }

                if (isCircleFound)
                    break;

            }

            if (isCircleFound) {
                upperRect1 = rotatedRects[matches[upperRecti].x];
                upperRect2 = rotatedRects[matches[upperRecti].y];

                ellipse(displayImg, upperRect1, Scalar(0, 0, 255));
                ellipse(displayImg, upperRect2, Scalar(0, 0, 255));
            }
            else {
                ROS_INFO("imbedded: %d, lower: %d", 
                         circleCandidatesFromImbedded.size(),
                         circleCandidatesfromLower.size());
            }
            */


            /*
            for (i = 0; i < matches.size(); i++) {
                ellipse(displayImg, rotatedRects[matches[i].x], Scalar(0, 0, 255));
                ellipse(displayImg, rotatedRects[matches[i].y], Scalar(0, 0, 255));
            }
            */

        }

        // draw stuff
        //polylines(displayImg, contours, true, Scalar(255, 0, 0), 1, 8, 0);
        /*
        for (i = 0; i < circles.size(); i++) {
            circle(displayImg, Point(circles[i][0]+drag0_.x, circles[i][1]+drag0_.y), circles[i][2], Scalar(0, 0, 255));
        }
        */
    }

    if (!displayImg.empty()) {
        imshow(RAWVID, displayImg);
    }
    if (!procImg.empty()) {
        imshow(PROCVID, procImg);
    }
    waitKey(20);

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
