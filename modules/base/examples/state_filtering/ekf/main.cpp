//---------------------------------------------------------------------------------------------------------------------
//  mico
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

#include <mico/base/state_filtering/ExtendedKalmanFilter.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

class Simple2DEKF: public mico::ExtendedKalmanFilter<float,4,2> {
protected:
    //---------------------------------------------------------------------------------------------------
    void updateJf(const double _incT){
        mJf.setIdentity();
        mJf.block<2,2>(0,2) = Eigen::Matrix<float,2,2>::Identity()*_incT;
    }

    //---------------------------------------------------------------------------------------------------
    void updateHZk(){
        mHZk = mXak.block<2,1>(0,0);
    }

    //---------------------------------------------------------------------------------------------------
    void updateJh(){
        mJh.setIdentity();
    }
};

int main(int _argc, char **_argv){

    const float NOISE_LEVEL = 0.1;

    Eigen::Matrix<float, 4, 4> Q_; // State covariance
    Q_.setIdentity();   
    Q_.block<2,2>(0,0) *= 0.01;
    Q_.block<2,2>(2,2) *= 0.03;

    Eigen::Matrix<float, 2, 2> mR; // Observation covariance
    mR.setIdentity();   
    mR.block<2,2>(0,0) *= NOISE_LEVEL*2;

    Eigen::Matrix<float, 4,1> x0;
    x0 <<   1,0,    // (x,y)
            0,0;    // (v_x, v_y)
    
    Simple2DEKF ekf;

    ekf.setUpEKF(Q_, mR, x0);
    cv::Mat map = cv::Mat::zeros(cv::Size(300, 300), CV_8UC3);

    cv::namedWindow("display", CV_WINDOW_FREERATIO);
    cv::Point2f prevObs(250, 150), prevState(250, 150);

    float fakeTimer = 0;
    
    while(true){
        Eigen::Matrix<float, 2,1> z;    // New observation
        z <<    0.5 + cos(fakeTimer*1)*0.5 + ((double)rand())/RAND_MAX*NOISE_LEVEL,
                0.5 + sin(fakeTimer*1)*0.5 + ((double)rand())/RAND_MAX*NOISE_LEVEL;

        fakeTimer += 0.03;

        ekf.stepEKF(z, 0.03);

        Eigen::Matrix<float,4,1> filteredX = ekf.state();

        cv::Point2f currentObs(
            z[0]*200 + 50,
            z[1]*200 + 50
        );
        cv::Point2f currentState(
            filteredX[0]*200 + 50,
            filteredX[1]*200 + 50
        );

        cv::line(map, prevObs, currentObs, cv::Scalar(100,100,255));
        cv::line(map, prevState, currentState, cv::Scalar(0,255,0),2);

        prevObs = currentObs;
        prevState = currentState;
        
        cv::imshow("display", map);
        cv::waitKey(30);

    }

}