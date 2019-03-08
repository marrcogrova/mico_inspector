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
#include <geometry_msgs/TwistStamped.h>
#include <thread>        
#include <mutex>


//			           1 2 3  4 5   6   7  8  9  10  11  12 13  14   15   16
// State variable x = [p q r Lp Lr Ldr Lda Np Nr Ndr Nda Mq Mde Mdth Ldth Ndth]lambda=correlation time bias
//                           1 2 3
// Observation variable z = [p q r] a=acelerometro m=magnetometro
//

float p=0, q=0, r=0;
float pnew=0, qnew=0, rnew=0;
float Ixx=0.0, Iyy=0.0, Izz=0.0, Ixz=0.0;
std::mutex mtx_com;
// reading angular velocity
void accel_Callback(const geometry_msgs::TwistStamped &msgvelocity)
{   pnew=msgvelocity.angular_velocity.x;
    qnew=msgvelocity.angular_velocity.y;
    rnew=msgvelocity.angular_velocity.z;
}


class EkfPose: public rgbd::ExtendedKalmanFilter<float,10,4> {
private:
const float lambda=0.0;

protected:
    //---------------------------------------------------------------------------------------------------
    void updateJf(const double _incT){
	float p=0.0, q=0.0, r=0.0, Lp, Lr, Ldr, Lda, Np, Nr, Ndr, Nda, Mq, Mde, Mdth, Ldth, Ndth;
	//señales de actuacción
	float dr,da,dth,de;/// identificar son necesarios

	 p = mXak(0,0);
	 q = mXak(1,0);
	 r = mXak(2,0);
	 Lp = mXak(3,0);
	 Lr = mXak(4,0);
	 Ldr = mXak(5,0);
	 Lda = mXak(6,0);
	 Np = mXak(7,0);
	 Nr = mXak(8,0);
	 Ndr = mXak(9,0);
	 Nda = mXak(10,0);
	 Mq = mXak(11,0);
	 Mde = mXak(12,0);
	 Mdth = mXak(13,0);
	 Ldth = mXak(14,0);
	 Ndth = mXak(15,0);
	
	 
	/// fila 1  
	mJf.setIdentity();
	mJf(0,0) = 1+((((Ixz*(Ixx-Iyy+Izz))/(Ixx*Izz-Ixz*Ixz))*q)+((Izz*Lp+Ixz*Ndr)/(Ixx*Izz-Ixz*Ixz)))*_incT+((((Ixz*(Ixx-Iyy+Izz))/(Ixx*Izz-Ixz*Ixz))*q)+((Izz*Lp+Ixz*Ndr)/(Ixx*Izz-Ixz*Ixz)))*_incT*((((Ixz*(Ixx-Iyy+Izz))/(Ixx*Izz-Ixz*Ixz))*q)+((Izz*Lp+Ixz*Ndr)/(Ixx*Izz-Ixz*Ixz)))*_incT/2;
	mJf(0,1) = 1+((((Ixz*(Ixx-Iyy+Izz))/(Ixx*Izz-Ixz*Ixz))*p)+(((Izz*(Iyy-Izz)-Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*r))*_incT+((((Ixz*(Ixx-Iyy+Izz))/(Ixx*Izz-Ixz*Ixz))*p)+(((Izz*(Iyy-Izz)-Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*r))*_incT/2*((((Ixz*(Ixx-Iyy+Izz))/(Ixx*Izz-Ixz*Ixz))*p)+(((Izz*(Iyy-Izz)-Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*r))*_incT;
	mJf(0,2) = 1+((((Izz*(Iyy-Izz)-Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*q)+((Izz*Lr+Ixz*Nr)/(Ixx*Izz-Ixz*Ixz)))*_incT+((((Izz*(Iyy-Izz)-Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*q)+((Izz*Lr+Ixz*Nr)/(Ixx*Izz-Ixz*Ixz)))*_incT/2*((((Izz*(Iyy-Izz)-Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*q)+((Izz*Lr+Ixz*Nr)/(Ixx*Izz-Ixz*Ixz)))*_incT;
	mJf(0,3) = 1+((Izz/(Ixx*Izz-Ixz*Ixz))*p)*_incT+((Izz/(Ixx*Izz-Ixz*Ixz))*p)*_incT/2*((Izz/(Ixx*Izz-Ixz*Ixz))*p)*_incT;
	mJf(0,4) = 1+((Izz/(Ixx*Izz-Ixz*Ixz))*r)*_incT+((Izz/(Ixx*Izz-Ixz*Ixz))*r)*_incT/2*((Izz/(Ixx*Izz-Ixz*Ixz))*r)*_incT;
	mJf(0,5) = 1+(((Izz*Lp+Ixz*Np)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT+(((Izz*Lp+Ixz*Np)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT/2*(((Izz*Lp+Ixz*Np)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT;
	mJf(0,6) = 1+(((Izz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT+(((Izz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT/2*(((Izz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT;
	mJf(0,7) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*p)*_incT++(((Ixz)/(Ixx*Izz-Ixz*Ixz))*p)*_incT/2*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*p)*_incT;
	mJf(0,8) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*r)*_incT+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*r)*_incT/2*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*r)*_incT;
	mJf(0,9) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT++(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT/2*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT;
	mJf(0,10) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT/2*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT;
	mJf(0,11) = 0;
	mJf(0,12) = 0;
	mJf(0,13) = 0;
	mJf(0,14) = 1+(((Izz)/(Ixx*Izz-Ixz*Ixz)))*_incT+(((Izz)/(Ixx*Izz-Ixz*Ixz)))*_incT/2*(((Izz)/(Ixx*Izz-Ixz*Ixz)))*_incT;
	mJf(0,15) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT/2*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT;
	// fila 2
	mJf(1,0) = 1+(((Izz-Ixx)/Iyy)*r-((Ixz)/Iyy)*2*p)*_incT+(((Izz-Ixx)/Iyy)*r-((Ixz)/Iyy)*2*p)*_incT/2*(((Izz-Ixx)/Iyy)*r-((Ixz)/Iyy)*2*p)*_incT;
	mJf(1,1) = 1+(Mq/Iyy)*_incT+(Mq/Iyy)*_incT+(Mq/Iyy)*_incT+(Mq/Iyy)*_incT*(Mq/Iyy)*_incT+(Mq/Iyy)*_incT/2;
	mJf(1,2) = 1+(((Izz-Ixx)/Iyy)*p-((Ixz)/Iyy)*2*r)*_incT+(((Izz-Ixx)/Iyy)*p-((Ixz)/Iyy)*2*r)*_incT*(((Izz-Ixx)/Iyy)*p-((Ixz)/Iyy)*2*r)*_incT/2;
	mJf(1,3) = 0;
	mJf(1,4) = 0;
	mJf(1,5) = 0;
	mJf(1,6) = 0;
	mJf(1,7) = 0;
	mJf(1,8) = 0;
	mJf(1,9) = 0;
	mJf(1,10) = 0;
	mJf(1,11) = 1+(q/Iyy)*_incT+(q/Iyy)*_incT+(q/Iyy)*_incT+(q/Iyy)*_incT*(q/Iyy)*_incT+(q/Iyy)*_incT/2;
	mJf(1,12) = 1+(de/Iyy)*_incT+(de/Iyy)*_incT+(de/Iyy)*_incT+(de/Iyy)*_incT*(de/Iyy)*_incT+(de/Iyy)*_incT/2;
	mJf(1,13) = 1+(dth/Iyy)*_incT+(dth/Iyy)*_incT/2+(dth/Iyy)*_incT+(dth/Iyy)*_incT*(dth/Iyy)*_incT+(dth/Iyy)*_incT/2;
	mJf(1,14) = 0;
	mJf(1,15) = 0;
	// fila 3
	mJf(2,0) = 1+(((Ixx*(Ixx-Izz)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*q+((Ixx*Np+Ixz*Lp)/(Ixx*Izz-Ixz*Ixz)))*_incT+(((Ixx*(Ixx-Izz)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*q+((Ixx*Np+Ixz*Lp)/(Ixx*Izz-Ixz*Ixz)))*_incT*(((Ixx*(Ixx-Izz)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*q+((Ixx*Np+Ixz*Lp)/(Ixx*Izz-Ixz*Ixz)))*_incT/2;
	mJf(2,1) = 1+(((Ixx*(Ixx-Izz)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*p+(((Ixz*(Iyy-Ixx-Izz))/(Ixx*Izz-Ixz*Ixz))*r))*_incT+(((Ixx*(Ixx-Izz)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*p+(((Ixz*(Iyy-Ixx-Izz))/(Ixx*Izz-Ixz*Ixz))*r))*_incT*(((Ixx*(Ixx-Izz)+Ixz*Ixz)/(Ixx*Izz-Ixz*Ixz))*p+(((Ixz*(Iyy-Ixx-Izz))/(Ixx*Izz-Ixz*Ixz))*r))*_incT/2;
	mJf(2,2) = 1+(((Ixz*(Iyy-Ixx-Izz))/(Ixx*Izz-Ixz*Ixz))*q+((Ixx*Nr+Ixx*Lr)/(Ixx*Izz-Ixz*Ixz)))*_incT+(((Ixz*(Iyy-Ixx-Izz))/(Ixx*Izz-Ixz*Ixz))*q+((Ixx*Nr+Ixx*Lr)/(Ixx*Izz-Ixz*Ixz)))*_incT*(((Ixz*(Iyy-Ixx-Izz))/(Ixx*Izz-Ixz*Ixz))*q+((Ixx*Nr+Ixx*Lr)/(Ixx*Izz-Ixz*Ixz)))*_incT/2;
	mJf(2,3) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*p)*_incT+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*p)*_incT*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*p)*_incT/2;
	mJf(2,4) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*r)*_incT+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*r)*_incT*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*r)*_incT/2;
	mJf(2,5) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT/2;
	mJf(2,6) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*da)*_incT/2;
	mJf(2,7) = 1+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*p)*_incT+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*p)*_incT*(((Ixx)/(Ixx*Izz-Ixz*Ixz))*p)*_incT/2;
	mJf(2,8) = 1+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*r)*_incT+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*r)*_incT*(((Ixx)/(Ixx*Izz-Ixz*Ixz))*r)*_incT/2;
	mJf(2,9) = 1+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT*(((Ixx)/(Ixx*Izz-Ixz*Ixz))*dr)*_incT/2;
	mJf(2,10) =1+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*da)*_incT+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*da)*_incT*(((Ixx)/(Ixx*Izz-Ixz*Ixz))*da)*_incT/2;
	mJf(2,11) = 0;
	mJf(2,12) = 0;
	mJf(2,13) = 0;
	mJf(2,14) = 1+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT+(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT*(((Ixz)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT/2;
	mJf(2,15) = 1+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT+(((Ixx)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT*(((Ixx)/(Ixx*Izz-Ixz*Ixz))*dth)*_incT/2;
	// fila 4
	mJf(3,0) = 0; 
	mJf(3,1) = 0; 
	mJf(3,2) = 0;	
	mJf(3,3) = 1;
	mJf(3,4) = 0;
	mJf(3,5) = 0;
	mJf(3,6) = 0;
	mJf(3,7) = 0;
	mJf(3,8) = 0;
	mJf(3,9) = 0;
	mJf(3,10) = 0;
	mJf(3,11) = 0;
	mJf(3,12) = 0;
	mJf(3,13) = 0;
	mJf(3,14) = 0;
	mJf(3,15) = 0;
	// fila 5
	mJf(4,0) = 0;
	mJf(4,1) = 0;
	mJf(4,2) = 0;
	mJf(4,3) = 0;
	mJf(4,4) = 1;
	mJf(4,5) = 0;
	mJf(4,6) = 0;
	mJf(4,7) = 0;
	mJf(4,8) = 0;
	mJf(4,9) = 0;
	mJf(4,10) = 0;
	mJf(4,11) = 0;
	mJf(4,12) = 0;
	mJf(4,13) = 0;
	mJf(4,14) = 0;
	mJf(4,15) = 0;
	// fila 6
	mJf(5,0) = 0;
	mJf(5,1) = 0;
	mJf(5,2) = 0;
	mJf(5,3) = 0;
	mJf(5,4) = 0;
	mJf(5,5) = 1;
	mJf(5,6) = 0;
	mJf(5,7) = 0;
	mJf(5,8) = 0;
	mJf(5,9) = 0;
	mJf(5,10) = 0;
	mJf(5,11) = 0;
	mJf(5,12) = 0;
	mJf(5,13) = 0;
	mJf(5,14) = 0;
	mJf(5,15) = 0;
	// fila 7
	mJf(6,0) = 0;
	mJf(6,1) = 0;
	mJf(6,2) = 0;
	mJf(6,3) = 0;
	mJf(6,4) = 0;
	mJf(6,5) = 0;
	mJf(6,6) = 1;
	mJf(6,7) = 0;
	mJf(6,8) = 0;
	mJf(6,9) = 0;
	mJf(6,10) = 0;
	mJf(6,11) = 0;
	mJf(6,12) = 0;
	mJf(6,13) = 0;
	mJf(6,14) = 0;
	mJf(6,15) = 0;
	// fila 8
	mJf(7,0) = 0;
	mJf(7,1) = 0;
	mJf(7,2) = 0;
	mJf(7,3) = 0;
	mJf(7,4) = 0;
	mJf(7,5) = 0;
	mJf(7,6) = 0;
	mJf(7,7) = 1;
	mJf(7,8) = 0;
	mJf(7,9) = 0;
	mJf(7,10) = 0;
	mJf(7,11) = 0;
	mJf(7,12) = 0;
	mJf(7,13) = 0;
	mJf(7,14) = 0;
	mJf(7,15) = 0;
	// fila 9
	mJf(8,0) = 0;
	mJf(8,1) = 0;
	mJf(8,2) = 0;
	mJf(8,3) = 0;
	mJf(8,4) = 0;
	mJf(8,5) = 0;
	mJf(8,6) = 0;
	mJf(8,7) = 0;
	mJf(8,8) = 1;
	mJf(8,9) = 0;
	mJf(8,10) = 0;
	mJf(8,11) = 0;
	mJf(8,12) = 0;
	mJf(8,13) = 0;
	mJf(8,14) = 0;
	mJf(8,15) = 0;
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
	mJf(9,9) = 1;
	mJf(9,10) = 0;
	mJf(9,11) = 0;
	mJf(9,12) = 0;
	mJf(9,13) = 0;
	mJf(9,14) = 0;
	mJf(9,15) = 0;
	// fila 11
	mJf(10,0) = 0;
	mJf(10,1) = 0;
	mJf(10,2) = 0;
	mJf(10,3) = 0;
	mJf(10,4) = 0;
	mJf(10,5) = 0;
	mJf(10,6) = 0;
	mJf(10,7) = 0;
	mJf(10,8) = 0;
	mJf(10,9) = 0;
	mJf(10,10) = 1;
	mJf(10,11) = 0;
	mJf(10,12) = 0;
	mJf(10,13) = 0;
	mJf(10,14) = 0;
	mJf(10,15) = 0;
	// fila 12
	mJf(11,0) = 0;
	mJf(11,1) = 0;
	mJf(11,2) = 0;
	mJf(11,3) = 0;
	mJf(11,4) = 0;
	mJf(11,5) = 0;
	mJf(11,6) = 0;
	mJf(11,7) = 0;
	mJf(11,8) = 0;
	mJf(11,9) = 0;
	mJf(11,10) = 0;
	mJf(11,11) = 1;
	mJf(11,12) = 0;
	mJf(11,13) = 0;
	mJf(11,14) = 0;
	mJf(11,15) = 0;
	// fila 13
	mJf(12,0) = 0;
	mJf(12,1) = 0;
	mJf(12,2) = 0;
	mJf(12,3) = 0;
	mJf(12,4) = 0;
	mJf(12,5) = 0;
	mJf(12,6) = 0;
	mJf(12,7) = 0;
	mJf(12,8) = 0;
	mJf(12,9) = 0;
	mJf(12,10) = 0;
	mJf(12,11) = 0;
	mJf(12,12) = 1;
	mJf(12,13) = 0;
	mJf(12,14) = 0;
	mJf(12,15) = 0;
	// fila 14
	mJf(13,0) = 0;
	mJf(13,1) = 0;
	mJf(13,2) = 0;
	mJf(13,3) = 0;
	mJf(13,4) = 0;
	mJf(13,5) = 0;
	mJf(13,6) = 0;
	mJf(13,7) = 0;
	mJf(13,8) = 0;
	mJf(13,9) = 0;
	mJf(13,10) = 0;
	mJf(13,11) = 0;
	mJf(13,12) = 0;
	mJf(13,13) = 1;
	mJf(13,14) = 0;
	mJf(13,15) = 0;
	// fila 15
	mJf(14,0) = 0;
	mJf(14,1) = 0;
	mJf(14,2) = 0;
	mJf(14,3) = 0;
	mJf(14,4) = 0;
	mJf(14,5) = 0;
	mJf(14,6) = 0;
	mJf(14,7) = 0;
	mJf(14,8) = 0;
	mJf(14,9) = 0;
	mJf(14,10) = 0;
	mJf(14,11) = 0;
	mJf(14,12) = 0;
	mJf(14,13) = 0;
	mJf(14,14) = 1;
	mJf(14,15) = 0;
	// fila 16
	mJf(15,0) = 0;
	mJf(15,1) = 0;
	mJf(15,2) = 0;
	mJf(15,3) = 0;
	mJf(15,4) = 0;
	mJf(15,5) = 0;
	mJf(15,6) = 0;
	mJf(15,7) = 0;
	mJf(15,8) = 0;
	mJf(15,9) = 0;
	mJf(15,10) = 0;
	mJf(15,11) = 0;
	mJf(15,12) = 0;
	mJf(15,13) = 0;
	mJf(15,14) = 0;
	mJf(15,15) = 1;
	}

    //---------------------------------------------------------------------------------------------------
    void updateHZk(){
		float pak=0.0;
		float qak=0.0;
		float rak=0.0;
		pak = mXak(0,0);
	 	qak = mXak(1,0);
	 	rak = mXak(2,0);
	 	

        mHZk[0] = pak; 
    	mHZk[1] = qak;
		mHZk[2] = rak;
		
	

    }

    //---------------------------------------------------------------------------------------------------
    void updateJh(){
	// fila 1
    mJh.setIdentity();
	mJh(0,0) = 1;
	mJh(0,1) = 0;
	mJh(0,2) = 0;
	mJh(0,3) = 0;
	mJh(0,4) = 0;
	mJh(0,5) = 0;
	mJh(0,6) = 0;
	mJh(0,7) = 0;
	mJh(0,8) = 0;
	mJh(0,9) = 0;
	mJh(0,10) = 0;
	mJh(0,11) = 0;
	mJh(0,12) = 0;
	mJh(0,13) = 0;
	mJh(0,14) = 0;
	mJh(0,15) = 0;
	// fila 2
	mJh(1,0) = 0;
	mJh(1,1) = 1;
	mJh(1,2) = 0;
	mJh(1,3) = 0;
	mJh(1,4) = 0;
	mJh(1,5) = 0;
	mJh(1,6) = 0;
	mJh(1,7) = 0;
	mJh(1,8) = 0;
	mJh(1,9) = 0;
	mJh(1,10) = 0;
	mJh(1,11) = 0;
	mJh(1,12) = 0;
	mJh(1,13) = 0;
	mJh(1,14) = 0;
	mJh(1,15) = 0;
	// fila 3
	mJh(2,0) = 0;
	mJh(2,1) = 0;
	mJh(2,2) = 1;
	mJh(2,3) = 0;
	mJh(2,4) = 0;
	mJh(2,5) = 0;
	mJh(2,6) = 0;
	mJh(2,7) = 0;
	mJh(2,8) = 0;
	mJh(2,9) = 0;
	mJh(2,10) = 0;
	mJh(2,11) = 0;
	mJh(2,12) = 0;
	mJh(2,13) = 0;
	mJh(2,14) = 0;
	mJh(2,15) = 0;
    }
};

int main(int _argc, char **_argv){

    const float NOISE_LEVEL = 0.1;
	
	

	// starting comunication

	ros::init(_argc, _argv, "mainPose_pqr");
	ros::NodeHandle n;
	
		ros::Subscriber sub_accel = n.subscribe("/mavros/imu/data_raw", 2, accel_Callback);
		ros::Subscriber sub_mag = n.subscribe("/mavros/imu/mag", 2, mag_Callback);
		ros::spin();


    Eigen::Matrix<float, 16, 16> mQ; // State covariance
	// x = [p q r Lp Lr Ldr Lda Np Nr Ndr Nda Mq Mde Mdth Ldth Ndth]
    mQ.setIdentity();   
    mQ.block<3,3>(0,0) *= 0.01;
    mQ.block<13,13>(3,3) *= 0.03;
	

    Eigen::Matrix<float, 3, 3> mR; 
	//z = [p q r]
    mR.setIdentity();   
    mR(0,0) = NOISE_LEVEL*2;
	mR(1,1) = NOISE_LEVEL*2;
	mR(2,2) = NOISE_LEVEL*2;
	

    Eigen::Matrix<float, 16,1> x0; // condiciones iniciales 
	// x = [p q r Lp Lr Ldr Lda Np Nr Ndr Nda Mq Mde Mdth Ldth Ndth]
    x0 <<   0,0,0,        // (p q r)
            0,0,0,0,      // (Lp Lr Ldr Lda)
			0,0,0,0,      // (Np Nr Ndr Nda)
			0,0,0,        // (Mq Mde Mdth)
			0,0;          // (Ldth Ndth)
			
    EkfPose ekf;

    ekf.setUpEKF(mQ, mR, x0);
    cv::Mat map = cv::Mat::zeros(cv::Size(300, 300), CV_8UC3);

    cv::namedWindow("display", CV_WINDOW_FREERATIO);
    cv::Point2f prevObs(250, 150), prevState(250, 150);

    float fakeTimer = 0;
	int pcount=0, qcount=0, rcount=0;

    
    while(true){
		
			if(pnew!=p){
				pcount=1;
			}
			if(qnew!=q){
				qcount=1;
			}
			if(rnew!=r){
				rcount=1;
			}
			if((acount==1)&&(bcount==1)&&(ccount==1)&&(dcount==1)){
				mtx_com.lock();
				p=pnew;
				q=qnew;
				r=rnew;
				mtx_com.unlock();
				pcount=0;
				qcount=0;
				rcount=0;
			}
	
        Eigen::Matrix<float, 3,1> z;    // New observation
        z <<    p,
				q,
				r,
				
        fakeTimer += 0.03;

        ekf.stepEKF(z, 0.03);

        Eigen::Matrix<float,16,1> filteredX = ekf.state();

    }

}
