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

#include <mico/base/segmentation/color_clustering/ColorClustering.h>
#include <mico/base/segmentation/color_clustering/types/ColorSpaceHSV8.h>
#include <mico/base/segmentation/color_clustering/types/ccsCreation.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

int main(int _argc, char **_argv){

    if(_argc != 2){
        std::cout << "Bad input arguments, give a path to an image" << std::endl;
        return -1;
    }

    std::string filepath = _argv[1];
    cv::Mat image = cv::imread(filepath);

    if(image.rows == 0){
        std::cout << "Bad file path" << std::endl;
        return -1;
    }

    cv::Mat segmentedImage;

    // MULTIPLE COLOR SEGMENTATION
    cv::cvtColor(image, segmentedImage, CV_BGR2HSV);

    auto ccs  = mico::CreateHSVCS_8c(255,255,255);
    
    std::vector<mico::ImageObject> objects;
    mico::ColorClustering<uchar>(segmentedImage.data, segmentedImage.rows, segmentedImage.cols, 100, objects, *ccs);

    cv::cvtColor(segmentedImage, segmentedImage, CV_HSV2BGR);
    cv::imshow("Segmented image full HSV 8", segmentedImage);

    cv::Mat display = image.clone();
    for(auto &obj: objects){
        cv::Rect bb(    obj.centroid().x - obj.width()/2,
                        obj.centroid().y - obj.height()/2,
                        obj.width(),
                        obj.height());

        std::cout  << bb <<std::endl;

        cv::rectangle(display, bb, cv::Scalar(0,255,0),2);
    }

    cv::imshow("Detected objects full HSV 8", display);
    cv::waitKey();


    // SINGLE COLOR SEGMENTATION
    cv::cvtColor(image, segmentedImage, CV_BGR2HSV);

    ccs  = mico::createSingleClusteredSpace(    85,125,
                                                50, 255,
                                                50, 255,
                                                180,255,255,32
                                                );

    objects.clear();
    mico::ColorClustering<uchar>(segmentedImage.data, segmentedImage.rows, segmentedImage.cols, 100, objects, *ccs);

    cv::cvtColor(segmentedImage, segmentedImage, CV_HSV2BGR);
    cv::imshow("Segmented image full HSV 8", segmentedImage);

    display = image.clone();
    for(auto &obj: objects){
        cv::Rect bb(    obj.centroid().x - obj.width()/2,
                        obj.centroid().y - obj.height()/2,
                        obj.width(),
                        obj.height());

        std::cout  << bb <<std::endl;

        cv::rectangle(display, bb, cv::Scalar(0,255,0),2);
    }

    cv::imshow("Detected objects full HSV 8", display);
    cv::waitKey();

}