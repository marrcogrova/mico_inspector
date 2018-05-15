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

int main(int _argc, char** _argv){
    if(_argc != 2){
        std::cout << "Bad input arguments, usage: " << std::endl;
        std::cout << "\t" << _argv[0] << " PATH_TO_IMAGE" << std::endl;
    }
    std::cout << "Downloading weights" << std::endl;
    system("wget -nc http://www.vigus.org/owncloud/index.php/s/eHdiz7gvxfNmJk0/download -O yolov2-tiny-voc_900.weights");
    system("wget -nc http://www.vigus.org/owncloud/index.php/s/aNbaCPxCkCaJGKT/download -O yolov2-tiny-voc.cfg");

    std::cout << "Model downloaded"<<std::endl;

    rgbd::WrapperDarknet detector;
    detector.init("yolov2-tiny-voc.cfg", "yolov2-tiny-voc_900.weights");

    cv::Mat image = cv::imread(_argv[1]);

    auto detections = detector.detect(image);
    std::cout << "Num detections " << detections.size() << std::endl;
    for(auto &detection: detections){
        cv::Rect rec(detection[2], detection[3], detection[4] -detection[2], detection[5]-detection[3]);
        cv::rectangle(image, rec, cv::Scalar(0,255,0));
    }
    cv::imshow("result", image);
    cv::waitKey();
}
