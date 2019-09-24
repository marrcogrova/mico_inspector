

#include <streamers.h>
#include <block.h>

#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

int main(){
    
    Block block1;
    block1.registerCallback([&](std::vector<std::any> _data){
        cv::Mat image = std::any_cast<cv::Mat>(_data[0]).clone();
        cv::imshow("image1", image);
        cv::waitKey(3);
        std::cout << "--------------------" << std::endl;
    });

    PolicyAllRequired pol;
    block1.setPolicy(&pol);

    Block block2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    block2.registerCallback([&](std::vector<std::any> _data){
        cv::Mat image = std::any_cast<cv::Mat>(_data[0]).clone();
        std::vector<cv::KeyPoint> kps;
        detector->detect(image, kps);
        cv::drawKeypoints(image, kps,image);
        cv::imshow("image2", image);
        cv::waitKey(3);
        std::cout << "--------------------" << std::endl;
    });

    PolicyAllRequired pol2;
    block2.setPolicy(&pol2);
    

    std::vector<ostream*> streams;
    streams.push_back(new ostreamCamera());

    for(auto &osi: streams){
        osi->registerPolicy(&pol);
        osi->registerPolicy(&pol2);
    }

    for(auto &osi: streams){
        osi->start();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    for(auto &osi: streams){
        osi->stop();
    }

}