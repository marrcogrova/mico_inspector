

#include <mico/flow/streamers/streamers.h>
#include <mico/flow/pipelines/pipeline.h>
#include <mico/flow/policies/policies.h>

#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

using namespace mico;

int main(){
    
    Block block1;
    block1.registerCallback([&](std::vector<std::any> _data, std::vector<bool> _valid){
        if(_valid[0]){
            cv::Mat image = std::any_cast<cv::Mat>(_data[0]);
            cv::imshow("image1", image);
        }
        if(_valid[1]){
            cv::Mat image2 = std::any_cast<cv::Mat>(_data[1]);
            cv::imshow("image2", image2);
        }
        cv::waitKey(3);
        std::cout << "--------------------" << std::endl;
    });

    // PolicyAllRequired pol;
    PolicyAny pol;
    block1.setPolicy(&pol);
    

    OstreamCamera stream;
    stream.registerPolicy(&pol, 0);
    stream.registerPolicy(&pol, 1);

    stream.start();
    

    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    stream.stop();
}