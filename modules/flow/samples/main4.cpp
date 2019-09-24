

#include <streamers.h>
#include <block.h>

#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

#include <pipeline.h>

int main(){

    Block block;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    block.registerCallback([&](std::vector<std::any> _data){
        std::cout << std::any_cast<int>(_data[0]) << std::endl;
        cv::Mat image = std::any_cast<cv::Mat>(_data[1]).clone();
        std::vector<cv::KeyPoint> kps;
        detector->detect(image, kps);
        cv::drawKeypoints(image, kps,image);
        cv::imshow("image2", image);
        cv::waitKey(3);
        std::cout << "--------------------" << std::endl;
    });


    Pipeline pipe;
    pipe.build( "int", 
                "mono", 
                "all_policy",
                &block, 
                "visualization");

    pipe.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    
    pipe.stop();
}