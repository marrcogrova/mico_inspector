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



#ifndef RGBD_MAP3D_WORD_H_
#define RGBD_MAP3D_WORD_H_

#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>

namespace rgbd {
    struct Word{
    public:
        bool isInFrame(int _id){
            return std::find(frames.begin(), frames.end(), _id) != frames.end();
        }
        bool isInCluster(int _id){
            return std::find(clusters.begin(), clusters.end(), _id) != clusters.end();
        }

        cv::Point2f cvProjectionf(int _id){
            return cv::Point2f(projections[_id][0], projections[_id][1]);
        }

        cv::Point2d cvProjectiond(int _id){
            return cv::Point2d(projections[_id][0], projections[_id][1]);
        }

        cv::Point3f cvPointf(){
            return cv::Point3f(point[0], point[1], point[2]);
        }

        cv::Point3d cvPointd(){
            return cv::Point3d(point[0], point[1], point[2]);
        }

        template<typename PointType_>
        PointType_ pclPoint(){
            PointType_ p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            return p;
        }


    public:
        int id;
        std::vector<float> point;
        std::vector<int> frames;
        std::unordered_map<int, std::vector<float>> projections;
        std::unordered_map<int, int> idxInKf;
        cv::Mat descriptor;

        std::vector<int> clusters;
        // map[cluster][dataframe]=projections 
        std::map<int,std::map<int, std::vector<float>>> clusterProjections; 
        // umap[cluster][dataframe]=descriptor 
        std::unordered_map<int,std::unordered_map<int, cv::Mat>> clusterDescriptor; 

        bool optimized=false;
        friend std::ostream& operator<<(std::ostream& os, const Word& w);

        // Getters
        template<typename PointType_>
        PointType_ asPclPoint(){
            PointType_ pclPoint;
            pclPoint.x = point[0];
            pclPoint.y = point[1];
            pclPoint.z = point[2];
            return pclPoint;
        }
    };
}

#endif
