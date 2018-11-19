//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <string>
#include <unordered_map>

#include <rgbd_tools/object_detection/dnn/WrapperDarknet.h>
#include <fstream>
#include <chrono>

int main(int _argc, char** _argv){
    if(_argc != 3 && _argc != 4){
        std::cout << "Bad input arguments, usage: " << std::endl;
        std::cout << "\t" << _argv[0] << " [path_to_cfg] [path_to_weights] [path_to_folder_images|path_to_video|do not add to use webcam]" << std::endl;
        return -1;
    }
    std::cout << "Model downloaded"<<std::endl;

    rgbd::WrapperDarknet detector;
    detector.init(_argv[1], _argv[2]);

    cv::VideoCapture streamImages;
    if(_argc == 3)
        streamImages.open(0);
    if(_argc == 4)
        streamImages.open(_argv[3]);

    for(;;){
        cv::Mat image;
        streamImages >> image;
    
        if(image.rows == 0)
            break;

        auto t0 = std::chrono::high_resolution_clock::now();
        
        auto detections = detector.detect(image);
    
        std::cout << "Num detections " << detections.size() << std::endl;
        for(auto &detection: detections){
           cv::Rect rec(detection[2], detection[3], detection[4] -detection[2], detection[5]-detection[3]);
           cv::rectangle(image, rec, cv::Scalar(0,255,0));
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        float time  = float(std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count())/1e9;
        std::cout << "Algorithm took: " << time << ". FPS: " << 1/time << std::endl;
        cv::imshow("result", image);
        cv::waitKey(_argc == 3?10:0);
    }

    std::cout << "no more images in the queue" << std::endl;
    return -1;
}
