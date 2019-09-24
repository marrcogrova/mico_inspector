

#include <mico/flow/streamers/streamers.h>
#include <mico/flow/blocks/block.h>
#include <mico/flow/pipelines/pipeline.h>

#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

using namespace mico;

int main(){

    Block block;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    block.registerCallback([&](std::vector<std::any> _data){
        cv::Mat image = std::any_cast<cv::Mat>(_data[0]).clone();
        std::vector<cv::KeyPoint> kps;
        detector->detect(image, kps);
        cv::drawKeypoints(image, kps,image);
        cv::imshow("image2", image);
        cv::waitKey(3);
        std::cout << "--------------------" << std::endl;
    });


    Pipeline pipe;
    pipe.build( "mono", 
                "all_policy",
                &block, 
                "visualization");

    pipe.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    
    pipe.stop();
}