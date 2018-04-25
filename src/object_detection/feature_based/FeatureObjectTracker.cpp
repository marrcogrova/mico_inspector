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

#include <rgbd_tools/object_detection/feature_based/FeatureObjectTracker.h>
#include <string>
#include <iostream>

using namespace std;

namespace rgbd{
    bool FeatureObjectTracker::init(cjson::Json &_configFile){
        // Load feature model.
        if (!mModel.load(_configFile["model"]) || !mModel.init(_configFile["features"])) {
            std::cout << "[MAIN APPLICATION] Can't open feature model" << std::endl;
            return false;
        }
    
        mMaxLostFrames = (int) _configFile["maxLostFrames"];
    	mScaleFactorWindowLost = _configFile["scaleFactorWindow"];

        // Init EKF
        mQ = Eigen::Matrix<double,6,6>::Identity();
        mQ.block<3,3>(0,0) *= 0.01;
        mQ.block<3,3>(3,3) *= 0.2;
        mR = Eigen::Matrix<double,6,6>::Identity();
        mR.block<3,3>(0,0) *= 0.05;
        mR.block<3,3>(3,3) *= 1.0;
        mTimeStamp = std::chrono::high_resolution_clock::now();
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool FeatureObjectTracker::update(cv::Mat &_image, Eigen::Matrix4f &_pose){
        if(_image.rows == 0){
			std::cout << "Error, empty image." << std::endl;
			return false;
		}

        cv::Mat usedFrame;
        switch(mStatus){
            case AppStatus::Lost:
                mLastWindow = cv::Rect(0,0,_image.cols,_image.rows);
                break;
            case AppStatus::Found:
                std::cout << "Status FOUND: Cropping image: (" +std::to_string(mLastWindow.x) +", " + std::to_string(mLastWindow.y) + ") " + std::to_string(mLastWindow.width) + "x" +std::to_string(mLastWindow.height) << std::endl;;
                usedFrame = _image(mLastWindow).clone();
                break;
            default:    // Not defined
                return false;
        }
        
        std::vector<cv::Point2f> inliers;
        cv::Mat position, orientation;
		if(mModel.find(usedFrame, mIntrinsics, mDistCoeff, position, orientation, inliers, mLastWindow)){
            std::cout << "Found "+std::to_string(inliers.size()) + "inliers" << std::endl;;
            std::cout << "Object position (Camara CS): ["  + std::to_string(position.at<double>(0))
                                                                        + ", " + std::to_string(position.at<double>(1))
                                                                        + ", " + std::to_string(position.at<double>(2))
                                                                        + "] and orientation (Camara CS): ["
                                                                        + std::to_string(orientation.at<double>(0))
                                                                        + ", " + std::to_string(orientation.at<double>(1))
                                                                        + ", " + std::to_string(orientation.at<double>(2)) + "]" << std::endl;

            switch(mStatus){
                case AppStatus::Lost:
                    {   // Init EKF
                    Eigen::Matrix<double, 6,1> x0;
                    x0 <<   position.at<double>(0),
                            position.at<double>(1),
                            position.at<double>(2),
                            orientation.at<double>(0),
                            orientation.at<double>(1),
                            orientation.at<double>(2);
                    mEKF.setUpEKF(mQ, mR, x0);
                    break;
                    }
                case AppStatus::Found:
                    {  // Update EKF
                    Eigen::Matrix<double, 6,1> newPosition;
                    newPosition <<  position.at<double>(0),
                                    position.at<double>(1),
                                    position.at<double>(2),
                                    orientation.at<double>(0),
                                    orientation.at<double>(1),
                                    orientation.at<double>(2);
                    double incT = (double) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-mTimeStamp).count();
                    mTimeStamp = std::chrono::high_resolution_clock::now();
                    mEKF.stepEKF(newPosition, incT);
                    break;
                    }
                default:    // Not defined
                    return false;
            }

            auto filteredPosition = mEKF.state();
            position.at<double>(0) = filteredPosition(0,0);
            position.at<double>(1) = filteredPosition(1,0);
            position.at<double>(2) = filteredPosition(2,0);
            orientation.at<double>(0) = filteredPosition(3,0);
            orientation.at<double>(1) = filteredPosition(4,0);
            orientation.at<double>(2) = filteredPosition(5,0);

            std::cout << "EKF position (Camara CS): ["  + std::to_string(position.at<double>(0))
                                                                        + ", " + std::to_string(position.at<double>(1))
                                                                        + ", " + std::to_string(position.at<double>(2))
                                                                        + "] and orientation (Camara CS): ["
                                                                        + std::to_string(orientation.at<double>(0))
                                                                        + ", " + std::to_string(orientation.at<double>(1))
                                                                        + ", " + std::to_string(orientation.at<double>(2)) + "]" << std::endl;

            // Change status and crop window
            mStatus = AppStatus::Found;
            computeNextWindow(inliers);
            mLastWindow &= cv::Rect(0, 0, _image.cols, _image.rows);

        }else{
            increaseSearchWindow(_image.cols, _image.rows);
        }

    }

    //-----------------------------------------------------------------------------------------------------------------
    void FeatureObjectTracker::drawCoordinate(const cv::Mat &_position, const cv::Mat &_rotation, cv::Mat &_image) {

        cv::Mat matRot;
        Rodrigues(_rotation, matRot);

        cv::Mat origin(3,1,CV_64F); origin.at<double>(0) = 0; origin.at<double>(1) = 0;origin.at<double>(2) = 0;
        cv::Mat axisX(3,1,CV_64F); axisX.at<double>(0) = 0.05; axisX.at<double>(1) = 0; axisX.at<double>(2) = 0;
        cv::Mat axisY(3,1,CV_64F); axisY.at<double>(0) = 0; axisY.at<double>(1) = 0.05; axisY.at<double>(2) = 0;
        cv::Mat axisZ(3,1,CV_64F); axisZ.at<double>(0) = 0; axisZ.at<double>(1) = 0; axisZ.at<double>(2) = 0.05;

        origin = _position + matRot*origin;
        axisX = _position + matRot*axisX;
        axisY = _position + matRot*axisY;
        axisZ = _position + matRot*axisZ;

        std::vector<cv::Point3f> axisPoints3d;
        axisPoints3d.push_back(cv::Point3f((float) origin.at<double>(0), (float) origin.at<double>(1), (float) origin.at<double>(2)));
        axisPoints3d.push_back(cv::Point3f((float) axisX.at<double>(0), (float) axisX.at<double>(1), (float) axisX.at<double>(2)));
        axisPoints3d.push_back(cv::Point3f((float) axisY.at<double>(0), (float) axisY.at<double>(1), (float) axisY.at<double>(2)));
        axisPoints3d.push_back(cv::Point3f((float) axisZ.at<double>(0), (float) axisZ.at<double>(1), (float) axisZ.at<double>(2)));


        cv::Mat cameraOrigin = cv::Mat::zeros(3,1, CV_32F);
        cv::Mat cameraOrientation = cv::Mat::zeros(3,1,CV_32F);
        std::vector<cv::Point2f> axisPoints2d;
        projectPoints(axisPoints3d, cameraOrientation, cameraOrigin, mIntrinsics, mDistCoeff, axisPoints2d);

        line(_image, axisPoints2d[0], axisPoints2d[1], cv::Scalar(0,0,255), 10);
        line(_image, axisPoints2d[0], axisPoints2d[2], cv::Scalar(0,255,0), 10);
        line(_image, axisPoints2d[0], axisPoints2d[3], cv::Scalar(255,0,0), 10);

    }

    //-----------------------------------------------------------------------------------------------------------------
    void FeatureObjectTracker::computeNextWindow(const std::vector<cv::Point2f> &_points) {
        float minX=99999, minY=99999, maxX=-1, maxY=-1;
        for(auto &p: _points){
            p.x < minX ? minX = p.x:0.0f;
            p.y < minY ? minY = p.y:0.0f;
            p.x > maxX ? maxX = p.x:0.0f;
            p.y > maxY ? maxY = p.y:0.0f;
        }
        minX *= 0.75f;   // 666 TODO: config file
        minY *= 0.75f;
        maxX *= 1.25f;
        maxY *= 1.25f;

        mLastWindow = cv::Rect(minX, minY, maxX-minX, maxY-minY);
    }

    //-----------------------------------------------------------------------------------------------------------------
    void FeatureObjectTracker::increaseSearchWindow(int _width, int _height) {
        mNumLostFrames++;
        if (mNumLostFrames > mMaxLostFrames) {
            mLastWindow = cv::Rect(0, 0, _width, _height);
            mStatus = AppStatus::Lost;
        }else{
            unsigned incPxWidth = (unsigned) mLastWindow.width*mScaleFactorWindowLost;
            unsigned incPxHeight = (unsigned) mLastWindow.height*mScaleFactorWindowLost;

            mLastWindow.x -= incPxWidth / 2;
            mLastWindow.y -= incPxHeight / 2;
            mLastWindow.width += incPxWidth;
            mLastWindow.height += incPxHeight;

            mLastWindow &= cv::Rect(0, 0, _width, _height);
        }
    }
}