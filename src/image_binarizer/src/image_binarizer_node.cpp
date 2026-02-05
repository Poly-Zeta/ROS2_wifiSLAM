#include "image_binarizer/image_binarizer.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

ImageBinarizer::ImageBinarizer(const std::string & node_name,const rclcpp::NodeOptions & node_options) : rclcpp::Node(node_name,node_options){

    image_topic_ =this->declare_parameter<std::string>("image_topic","/image_raw");

    binImage_topic_ =this->declare_parameter<std::string>("bin_image_topic","/image_binary");

    image_sub_=this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
            &ImageBinarizer::imageCallback,
            this,
            std::placeholders::_1
        )
    );

    binImage_pub_=this->create_publisher<sensor_msgs::msg::Image>(
        binImage_topic_,
        rclcpp::SensorDataQoS()
    );
}


static inline void setBorderZero(cv::Mat& img, int b = 2){
    CV_Assert(img.type() == CV_8UC1);
    img.rowRange(0, b).setTo(0);
    img.rowRange(img.rows - b, img.rows).setTo(0);
    img.colRange(0, b).setTo(0);
    img.colRange(img.cols - b, img.cols).setTo(0);
}


void ImageBinarizer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg){

    cv::Mat cv_image =cv_bridge::toCvCopy(msg,"bgr8")->image;

    cv::Mat cv_grayScale;
    cv::Mat cv_illum;
    cv::Mat cv_dark;
    cv::cvtColor(cv_image,cv_grayScale,cv::COLOR_BGR2GRAY);
//    cv::medianBlur(cv_grayScale,cv_grayScale,7);
    //cv::bitwise_not(cv_grayScale,cv_dark);
    //cv_dark=cv_illum-cv_grayScale;
//    cv::absdiff(cv_grayScale,cv_illum,cv_dark);

//    cv::morphologyEx(
//        cv_dark,
//        cv_dark,
//        cv::MORPH_BLACKHAT,
//        cv::getStructuringElement(cv::MORPH_RECT,cv::Size(21,21)),
//        cv::Point(-1,-1),
//        1
//    );
    //cv::Mat mask;
    //cv::threshold(cv_dark,mask,10,255,cv::THRESH_BINARY);


    cv::Mat cv_hsv;
    cv::cvtColor(cv_image,cv_hsv,cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_ch;
    cv::split(cv_hsv,hsv_ch);

    cv::Mat cv_out;
//    cv::normalize(cv_dark,cv_dark,0,255,cv::NORM_MINMAX,-1);
//    cv::normalize(hsv_ch[1],hsv_ch[1],0,255,cv::NORM_MINMAX,-1);
//    cv::addWeighted(cv_dark,0.5,hsv_ch[1],0.5,0,cv_out);
//    cv::max(cv_dark,hsv_ch[1],cv_out);

    cv::Mat cv_binary;

    //cv::threshold(cv_out,cv_binary,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU);
//    cv::threshold(cv_out,cv_binary,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::adaptiveThreshold(
        cv_grayScale,//dark,
        cv_binary,
        255,
        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY,
        31,
        5
    );
//    cv::Mat bin=cv::Mat::ones(cv_binary.size(),cv_binary.type())*255;
//    cv_binary.copyTo(bin,mask);
//
    cv::morphologyEx(
        cv_binary,
        cv_binary,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7)),
        cv::Point(-1,-1),
        1
    );
    cv::bitwise_not(cv_binary,cv_binary);
    cv::morphologyEx(
        cv_binary,
        cv_binary,
        cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)),
        cv::Point(-1,-1),
        1
    );
    //auto img_msg=cv_bridge::CvImage(msg->header,"mono8",cv_dark).toImageMsg();
    

//    cv::Mat element = cv::Mat::ones(3, 3, CV_8UC1);
//    cv::erode(cv_binary, cv_binary, element, cv::Point(-1, -1), 1);
//
//    cv::Mat regionMask=cv_binary.clone();
//    setBorderZero(regionMask, 2);
//
//
//    cv::Mat labels, stats, centroids;
//    int nLabels = cv::connectedComponentsWithStats(
//        regionMask, labels, stats, centroids, 8, CV_32S
//    );
//
//    int bestLabel = -1;
//    //double bestMean = 1e18;
//    double bestScore = 1e18;
//
//    const int minArea = 300;
//
//    const double globalMean = cv::mean(cv_grayScale)[0];
//
//    for (int l = 1; l < nLabels; ++l){
//        int area = stats.at<int>(l, cv::CC_STAT_AREA);
//        if (area < minArea) continue;
//
//        cv::Mat mask = (labels == l);
//
//        const double meanVal = cv::mean(cv_grayScale, mask)[0];
//
//        const double score = meanVal - globalMean;
//
//        if (score < bestScore){
//            bestScore = score;
//            bestLabel = l;
//        }
//
////        if (meanVal < bestMean)
////        {
////            bestMean = meanVal;
////            bestLabel = l;
////        }
//    }
//
//    cv::Mat out(cv_grayScale.size(), CV_8UC1, cv::Scalar(255));
//
//    if (bestLabel >= 0)
//    {
//        out.setTo(0, labels == bestLabel);
//    }
//
//  //  cv::Mat element = cv::Mat::ones(3, 3, CV_8UC1);
////    cv::erode(out, out, element, cv::Point(-1, -1), 1);

    //auto img_msg=cv_bridge::CvImage(msg->header,"mono8",out).toImageMsg();
    auto img_msg=cv_bridge::CvImage(msg->header,"mono8",cv_binary).toImageMsg();
    binImage_pub_->publish(*img_msg);
}
