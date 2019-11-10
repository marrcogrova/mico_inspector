

#include <streamers.h>
#include <block.h>

#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

int main(){
    
   
    flow::Block block;
    block.registerCallback([&](std::vector<std::any> _data){
                std::cout << std::any_cast<int>(_data[0]) << std::endl;
                std::cout << std::any_cast<std::string>(_data[1]) << std::endl;
                std::cout << std::any_cast<std::string>(_data[2]) << std::endl;
                std::cout << std::any_cast<int>(_data[3]) << std::endl;
                cv::Mat image = std::any_cast<cv::Mat>(_data[4]);
                cv::imshow("image", image);
                cv::waitKey(3);
                std::cout << "--------------------" << std::endl;
        });

    PolicyAllRequired pol;
    block.setPolicy(&pol);
    

    std::vector<ostream*> streams;
    streams.push_back(new ostreamInt());
    streams.push_back(new ostreamString());
    streams.push_back(new ostreamString());
    streams.push_back(new ostreamInt());
    streams.push_back(new ostreamCamera());

    for(auto &osi: streams){
        osi->registerPolicy(&pol);
    }

    for(auto &osi: streams){
        osi->start();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    for(auto &osi: streams){
        osi->stop();
    }

}
