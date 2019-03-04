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

#include <rgbd_tools/state_filtering/ExtendedKalmanFilter.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <thread>        
#include <mutex>


//			0  1 2  3  4  5  6   7   8   9
// State variable x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]lambda=correlation time bias
//
// Observation variable z = [xa ya za zm] a=acelerometro m=magnetometro
//

float a=0, b=0, c=0, d=0;
float anew=0, bnew=0, cnew=0, dnew=0;
std::mutex mtx_com;
void accel_Callback(const sensor_msgs::Imu &msgaccel)
{   anew=msgaccel.linear_acceleration.x;
    bnew=msgaccel.linear_acceleration.y;
    cnew=msgaccel.linear_acceleration.z;
}
void mag_Callback(const sensor_msgs::MagneticField &msgmag)
{
    dnew=msgmag.magnetic_field.z;
}
void synchronization(float ab,float bb, float cb, float db){//ab=abefore
	mtx_com.lock();
	a=ab;
	b=bb;
	c=cb;
	d=db;
	mtx_com.unlock();


}


class EkfPose: public rgbd::ExtendedKalmanFilter<float,10,4> {
private:
const float lambda=0.0;

protected:
    //---------------------------------------------------------------------------------------------------
    void updateJf(const double _incT){
	float q0=0.0;
	float q1=0.0;
	float q2=0.0;
	float q3=0.0;
	float Wi=0.0;
	float Wj=0.0;
	float Wk=0.0;
	 q0 = mXak(0,0);
	 q1 = mXak(1,0);
	 q2 = mXak(2,0);
	 q3 = mXak(3,0);
	 Wi = mXak(4,0);
	 Wj = mXak(5,0);
	 Wk = mXak(6,0);
	/// fila 1  
	mJf.setIdentity();
	mJf(0,0) = 1;
	mJf(0,1) = (_incT/2)*(-1)*Wi;
	mJf(0,2) = (_incT/2)*(-1)*Wj;
	mJf(0,3) = (_incT/2)*(-1)*Wk;
	mJf(0,4) = (_incT/2)*(-1)*q1;
	mJf(0,5) = (_incT/2)*(-1)*q2;
	mJf(0,6) = (_incT/2)*(-1)*q3;
	mJf(0,7) = 0;
	mJf(0,8) = 0;
	mJf(0,9) = 0;
	// fila 2
	mJf(1,0) = (_incT/2)*Wi;
	mJf(1,1) = 1;
	mJf(1,2) = (_incT/2)*Wk;
	mJf(1,3) = (_incT/2)*(-1)*Wj;
	mJf(1,4) = (_incT/2)*q0;
	mJf(1,5) = (_incT/2)*(-1)*q3;
	mJf(1,6) = (_incT/2)*q2;
	mJf(1,7) = 0;
	mJf(1,8) = 0;
	mJf(1,9) = 0;
	// fila 3
	mJf(2,0) = (_incT/2)*Wj;
	mJf(2,1) = (_incT/2)*(-1)*Wk;
	mJf(2,2) = 1;
	mJf(2,3) = (_incT/2)*Wi;
	mJf(2,4) = (_incT/2)*q3;
	mJf(2,5) = (_incT/2)*q0;
	mJf(2,6) = (_incT/2)*(-1)*q1;
	mJf(2,7) = 0;
	mJf(2,8) = 0;
	mJf(2,9) = 0;
	// fila 4
	mJf(3,0) = (_incT/2)*Wk;
	mJf(3,1) = (_incT/2)*Wj;
	mJf(3,2) = (_incT/2)*(-1)*Wi;
	mJf(3,3) = 1;
	mJf(3,4) = (_incT/2)*(-1)*q2;
	mJf(3,5) = (_incT/2)*q1;
	mJf(3,6) = (_incT/2)*q0;
	mJf(3,7) = 0;
	mJf(3,8) = 0;
	mJf(3,9) = 0;
	// fila 5
	mJf(4,0) = 0;
	mJf(4,1) = 0;
	mJf(4,2) = 0;
	mJf(4,3) = 0;
	mJf(4,4) = 0;
	mJf(4,5) = 0;
	mJf(4,6) = 0;
	mJf(4,7) = (-1);
	mJf(4,8) = 0;
	mJf(4,9) = 0;
	// fila 6
	mJf(5,0) = 0;
	mJf(5,1) = 0;
	mJf(5,2) = 0;
	mJf(5,3) = 0;
	mJf(5,4) = 0;
	mJf(5,5) = 0;
	mJf(5,6) = 0;
	mJf(5,7) = 0;
	mJf(5,8) = (-1);
	mJf(5,9) = 0;
	// fila 7
	mJf(6,0) = 0;
	mJf(6,1) = 0;
	mJf(6,2) = 0;
	mJf(6,3) = 0;
	mJf(6,4) = 0;
	mJf(6,5) = 0;
	mJf(6,6) = 0;
	mJf(6,7) = 0;
	mJf(6,8) = 0;
	mJf(6,9) = (-1);
	// fila 8
	mJf(7,0) = 0;
	mJf(7,1) = 0;
	mJf(7,2) = 0;
	mJf(7,3) = 0;
	mJf(7,4) = 0;
	mJf(7,5) = 0;
	mJf(7,6) = 0;
	mJf(7,7) = (1-lambda*_incT);
	mJf(7,8) = 0;
	mJf(7,9) = 0;
	// fila 9
	mJf(8,0) = 0;
	mJf(8,1) = 0;
	mJf(8,2) = 0;
	mJf(8,3) = 0;
	mJf(8,4) = 0;
	mJf(8,5) = 0;
	mJf(8,6) = 0;
	mJf(8,7) = 0;
	mJf(8,8) = (1-lambda*_incT);
	mJf(8,9) = 0;
	// fila 10
	mJf(9,0) = 0;
	mJf(9,1) = 0;
	mJf(9,2) = 0;
	mJf(9,3) = 0;
	mJf(9,4) = 0;
	mJf(9,5) = 0;
	mJf(9,6) = 0;
	mJf(9,7) = 0;
	mJf(9,8) = 0;
	mJf(9,9) = (1-lambda*_incT);
	}

    //---------------------------------------------------------------------------------------------------
    void updateHZk(){
		float q0=0.0;
		float q1=0.0;
		float q2=0.0;
		float q3=0.0;
		q0 = mXak(0,0);
	 	q1 = mXak(1,0);
	 	q2 = mXak(2,0);
	 	q3 = mXak(3,0);

        mHZk[0] = (-1)*2*(q1*q3-q0*q2);
    	mHZk[1] = (-1)*2*(q2*q3-q0*q1);
		mHZk[2] = (-1)*((q0*q0)-(q1*q1)-(q2*q2)-(q3*q3));
		mHZk[3] = atan2(2*((q0*q3)+(q1*q2)),1-2*((q2*q2)+(q3*q3)));
	

    }

    //---------------------------------------------------------------------------------------------------
    void updateJh(){
	float U=0.0;
	float U2=0.0;
	float Uf=0.0;
	float q0=0.0;
	float q1=0.0;
	float q2=0.0;
	float q3=0.0;

	q0 = mXfk(0,0);
	q1 = mXfk(1,0);
	q2 = mXfk(2,0);
	q3 = mXfk(3,0);
	// fila 1
    mJh.setIdentity();
	mJh(0,0) = 2*q2;
	mJh(0,1) = (-1)*2*q3;
	mJh(0,2) = 2*q0;
	mJh(0,3) = (-1)*2*q1;
	mJh(0,4) = 0;
	mJh(0,5) = 0;
	mJh(0,6) = 0;
	mJh(0,7) = 0;
	mJh(0,8) = 0;
	mJh(0,9) = 0;
	// fila 2
	mJh(1,0) = (-1)*2*q1;
	mJh(1,1) = (-1)*2*q0;
	mJh(1,2) = (-1)*2*q3;
	mJh(1,3) = (-1)*2*q2;
	mJh(1,4) = 0;
	mJh(1,5) = 0;
	mJh(1,6) = 0;
	mJh(1,7) = 0;
	mJh(1,8) = 0;
	mJh(1,9) = 0;
	// fila 3
	mJh(2,0) = (-1)*2*q0;
	mJh(2,1) = (-1)*2*q1;
	mJh(2,2) = (-1)*2*q2;
	mJh(2,3) = 2*q3;
	mJh(2,4) = 0;
	mJh(2,5) = 0;
	mJh(2,6) = 0;
	mJh(2,7) = 0;
	mJh(2,8) = 0;
	mJh(2,9) = 0;
	// fila 4

	 
	U= ((2*(q0*q3+q1*q2))/(1-2*((q2*q2+q3*q3))));
	U2= (1/(1+U*U));
	Uf= (1-2*(q2*q2+q3*q3));

	mJh(3,0) = (2*q3/Uf)*U2;
	mJh(3,1) = (2*q2/Uf)*U2;
	mJh(3,2) = (((2*q1*Uf)-2*(q0*q3+q1*q2)*(4*q2))/(Uf*Uf))*U2;
	mJh(3,3) = (((2*q0*Uf)-2*(q0*q3+q1*q2)*(4*q3))/(Uf*Uf))*U2;
	mJh(3,4) = 0;
	mJh(3,5) = 0;
	mJh(3,6) = 0;
	mJh(3,7) = 0;
	mJh(3,8) = 0;
	mJh(3,9) = 0;

    }
};

int main(int _argc, char **_argv){

    const float NOISE_LEVEL = 0.1;
	
	

	// starting comunication

	ros::init(_argc, _argv, "mainPose");
	ros::NodeHandle n;
	
		ros::Subscriber sub_accel = n.subscribe("/mavros/imu/data_raw", 2, accel_Callback);
		ros::Subscriber sub_mag = n.subscribe("/mavros/imu/mag", 2, mag_Callback);
		ros::spin();


    Eigen::Matrix<float, 10, 10> mQ; // State covariance
	// x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]
    mQ.setIdentity();   
    mQ.block<4,4>(0,0) *= 0.01;
    mQ.block<3,3>(4,4) *= 0.03;
	mQ.block<3,3>(6,6) *= 0.01;

    Eigen::Matrix<float, 4, 4> mR; // Observation covariance
	// Errrores en la medida medida de  nuestros sensores z = [xa ya za zm]
    mR.setIdentity();   
    mR(0,0) = NOISE_LEVEL*2;
	mR(1,1) = NOISE_LEVEL*2;
	mR(2,2) = NOISE_LEVEL*3;
	mR(3,3) = NOISE_LEVEL*4;

    Eigen::Matrix<float, 10,1> x0; // condiciones iniciales 
	// x = [q0 q1 q2 q3 wi wj wk xgi xgj xgk]
    x0 <<   1,0,0,1,    // (q00,q10,q20,q30)
            0,0,0,      // (wi0, wj0, wk0)
			0,0,0;      // (xgi0, xgj0, xgk0)
			
    EkfPose ekf;

    ekf.setUpEKF(mQ, mR, x0);
    cv::Mat map = cv::Mat::zeros(cv::Size(300, 300), CV_8UC3);

    cv::namedWindow("display", CV_WINDOW_FREERATIO);
    cv::Point2f prevObs(250, 150), prevState(250, 150);

    float fakeTimer = 0;
	int acount=0, bcount=0, ccount=0,dcount=0;

    
    while(true){
		
			if(anew!=a){
				acount=1;
			}
			if(bnew!=b){
				bcount=1;
			}
			if(cnew!=c){
				ccount=1;
			}
			if(dnew!=d){
				dcount=1;
			}
			if((acount==1)&&(bcount==1)&&(ccount==1)&&(dcount==1)){
				mtx_com.lock();
				a=anew;
				b=bnew;
				c=cnew;
				d=dnew;
				mtx_com.unlock();
				acount=0;
				bcount=0;
				ccount=0;
			}
	
        Eigen::Matrix<float, 4,1> z;    // New observation
        z <<    a,
				b,
				c,
				d;
				
        fakeTimer += 0.03;

        ekf.stepEKF(z, 0.03);

        Eigen::Matrix<float,10,1> filteredX = ekf.state();

    }

}
