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


#include <opencv2/opencv.hpp>
#include <vector>
#include <rgbd_tools/map3d/utils2d.h>

namespace rgbd{

    bool matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers,double _mk_nearest_neighbors,double _mFactorDescriptorDistance){
        std::vector<std::vector<cv::DMatch>> matches12, matches21;
        cv::BFMatcher featureMatcher;
        featureMatcher.knnMatch(_des1, _des2, matches12,_mk_nearest_neighbors);
        featureMatcher.knnMatch(_des2, _des1, matches21,_mk_nearest_neighbors);

        double max_dist = 0; double min_dist = 999999;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < _des1.rows; i++ ) {
            for(int j = 0; j < matches12[i].size(); j++){
                double dist = matches12[i][j].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
        }

        // symmetry test.
        for( int i = 0; i < _des1.rows; i++ )
            for(std::vector<cv::DMatch>::iterator it12 = matches12[i].begin(); it12 != matches12[i].end(); it12++){
                for( int j = 0; j < _des2.rows; j++ )
                    for(std::vector<cv::DMatch>::iterator it21 = matches21[j].begin(); it21 != matches21[j].end(); it21++){
                        if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                            if(it12->distance <= min_dist*_mFactorDescriptorDistance){
                                _inliers.push_back(*it12);
                            }
                            break;
                        }
                    }
            }

        /*
        // symmetry test.
        for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
            for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                    if(it12->distance <= min_dist*mFactorDescriptorDistance){
                        _inliers.push_back(*it12);
                    }
                    break;
                }
            }
        }
        */
        return true;
	}
}

