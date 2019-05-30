//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------tput
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

#include <rgbd_tools/utils/Graph2d.h>// Errores en la medida medida de  nuestros sensores z = [xa ya za Yaw]
#include <pcl/visualization/pcl_visualizer.h>
#include <serial/serial.h> 
//                       1    2    3     4  ac_yaw*sin(pitch)+ac_roll*cos(pitch)*cos(yaw)-ac_pitch*cos(pitch)*sin(yaw);       5        6      7       8        9			            
// State variable x = [roll pitch yaw vel_roll vel_pitch vel_yaw ac_roll ac_pitch ac_yaw] referencia mundo
//
// Observation variable z = [ax_new ay_new az_new] acelerometro refencia local
//

//// Captación de los quaternions
float q0new = 99, q1new = 99, q2new = 99, q3new = 99;
float wi_new=99, wj_new=99, wk_new=99;
float ax_new = 99, ay_new = 99, az_new = 99, xmag_new = 99,ymag_new=99, zmag_new=99;
float a = 99, b = 99, c = 99, d = 99, e = 99, f = 99;
const double PI  =3.141592653589793238463;
float g=9.81;
float mag=0.6;


////////////////// READING IMU-Pololy
serial::Serial  *mSerialPort = nullptr;
std::vector<std::string> observacion; 

std::mutex mtx_com;

////////////////////////////////////////////////////////Lectura del puerto serie y organización de datos
size_t split(const std::string &txt, std::vector<std::string> &strs, char ch) {
    size_t pos = txt.find( ch );
    size_t initialPos = 0;
    strs.clear();

    // Decompose statement
    while( pos != std::string::npos ) {
        strs.push_back( txt.substr( initialPos, pos - initialPos ) );
        initialPos = pos + 1;

        pos = txt.find( ch, initialPos );
    }

    // Add the last one
    strs.push_back( txt.substr( initialPos, std::min( pos, txt.size() ) - initialPos + 1 ) );

    return strs.size();
}
void serial_listen(){
    float size=0;
    //mtx_com.lock();
    while(size!=10){
        std::string resultRead = mSerialPort->readline(65536, "\r\n");
        std::cout << "LECTURA PUERTO SERIE:" << resultRead << std::endl;///////////// Nos da un vector pero con los números independientes   
        split(resultRead,observacion,'\t');
        size=observacion.size();
    }
    std::cout << "tamaño del vector:" << size << std::endl;
    std::cout << "Pos 1:" << observacion[0] << std::endl;
    std::cout << "Pos 2:" << observacion[1] << std::endl;
    wi_new=atof(observacion[1].c_str())*9.8/256;
    std::cout << "Pos 3:" << observacion[2] << std::endl;
    wj_new=atof(observacion[2].c_str())*9.8/256;
    std::cout << "Pos 4:" << observacion[3] << std::endl;
    wk_new=atof(observacion[3].c_str())*9.8/256;
    std::cout << "Pos 5:" << observacion[4] << std::endl;
    ax_new=atof(observacion[4].c_str());
    std::cout << "Pos 6:" << observacion[5] << std::endl;
    ay_new=atof(observacion[5].c_str());
    std::cout << "Pos 7:" << observacion[6] << std::endl;
    az_new=atof(observacion[6].c_str());
    std::cout << "Pos 8:" << observacion[7] << std::endl;
    xmag_new=atof(observacion[7].c_str());
    std::cout << "Pos 10:" << xmag_new << std::endl;
    std::cout << "Pos 9:" << observacion[8] << std::endl;
    ymag_new=atof(observacion[8].c_str());
    std::cout << "Pos 10:" << ymag_new << std::endl;
    std::cout << "Pos 10:" << observacion[9] << std::endl;
    zmag_new=atof(observacion[9].c_str());
    std::cout << "Pos 10:" << zmag_new << std::endl;
    //mtx_com.unlock();
    
}

/////////////////////////////// Actualización del valor del magnetometro
void actualiza_magnetometometer(){
	while (1)
	{
		if (mag<xmag_new)
		{
			mag=xmag_new;
		}
		
	}
	

}
class EkfEuler : public rgbd::ExtendedKalmanFilter<float, 9, 7>
{
  private:
	const float lambda = 1;

  protected:
	void updateJf(const double _incT)
	{
		mJf.setIdentity();
	  // Fila 1
		mJf(0,0)=1;
		mJf(0,1)=0;
		mJf(0,2)=0;
		mJf(0,3)=_incT;
		mJf(0,4)=0;
        mJf(0,5)=0;
		mJf(0,6)=(_incT*_incT)*(1.0/2.0);
		mJf(0,7)=0;
		mJf(0,8)=0;
		// Fila 2
		mJf(1,0)=0;
		mJf(1,1)=1;
		mJf(1,2)=0;
		mJf(1,3)=0;
		mJf(1,4)=_incT;
        mJf(1,5)=0;
		mJf(1,6)=0;
		mJf(1,7)=(_incT*_incT)*(1.0/2.0);
		mJf(1,8)=0;
		// Fila 3
		mJf(2,0)=0;
		mJf(2,1)=0;
		mJf(2,2)=1;
		mJf(2,3)=0;
		mJf(2,4)=0;
        mJf(2,5)=_incT;
		mJf(2,6)=0;
		mJf(2,7)=0;
		mJf(2,8)=(_incT*_incT)*(1.0/2.0);
		// Fila 4
		mJf(3,0)=0;
		mJf(3,1)=0;
		mJf(3,2)=0;
		mJf(3,3)=1;
		mJf(3,4)=0;
  	    mJf(3,5)=0;
		mJf(3,6)=_incT;
		mJf(3,7)=0;
		mJf(3,8)=0;
		// Fila 5
		mJf(4,0)=0;
		mJf(4,1)=0;
		mJf(4,2)=0;
		mJf(4,3)=0;
		mJf(4,4)=1;
        mJf(4,5)=0;
		mJf(4,6)=0;
		mJf(4,7)=_incT;
		mJf(4,8)=0;
		// Fila 6
		mJf(5,0)=0;
		mJf(5,1)=0;
		mJf(5,2)=0;
		mJf(5,3)=0;
		mJf(5,4)=0;
        mJf(5,5)=1;
		mJf(5,6)=0;
		mJf(5,7)=0;
        mJf(7,5)=0;
		mJf(5,8)=_incT;
		// Fila 7
		mJf(6,0)=0;
		mJf(6,1)=0;
		mJf(6,2)=0;
		mJf(6,3)=0;
		mJf(6,4)=0;
        mJf(6,5)=0;
		mJf(6,6)=1;
		mJf(6,7)=0;
		mJf(6,8)=0;
		// Fila 8
		mJf(7,0)=0;
		mJf(7,1)=0;
		mJf(7,2)=0;
		mJf(7,3)=0;
		mJf(7,4)=0;
        mJf(7,5)=0;
		mJf(7,6)=0;
		mJf(7,7)=1;
		mJf(7,8)=0;
		// Fila 9
		mJf(8,0)=0;
		mJf(8,1)=0;
		mJf(8,2)=0;
		mJf(8,3)=0;
		mJf(8,4)=0;
        mJf(8,5)=0;
		mJf(8,6)=0;
		mJf(8,7)=0;
		mJf(8,8)=1;
	}
	void updateHZk()
	{ 	
	/////////////// Accelerometer
		float roll=mXfk(0,0);
	  float pitch=mXfk(1,0);
		float yaw=mXfk(2,0);
		mHZk(0,0) =-g*sin(pitch);
		mHZk(1,0) =g*cos(pitch)*sin(roll);
		mHZk(2,0) =g*cos(pitch)*cos(roll);
	//////////////// Giroscope
		float v_roll=mXfk(3,0);
		float v_pitch=mXfk(4,0);
		float v_yaw=mXfk(5,0);
		mHZk(3,0) =-v_yaw*sin(pitch)+v_roll*cos(pitch)*cos(yaw)+v_pitch*cos(pitch)*sin(yaw);
		mHZk(4,0) =v_pitch*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw))-v_roll*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll))+v_yaw*cos(pitch)*sin(roll);
		mHZk(5,0) =-v_pitch*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw))+v_roll*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch))+v_yaw*cos(pitch)*cos(roll);
    //////////////// Magnetometer
		float mag=1.6;
		mHZk(6,0) =mag*cos(pitch)*cos(yaw);
		//mHZk(7,0) =-mag*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll));
		//mHZk(8,0)	=mag*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch));

	}

	void updateJh()
	{	
	///////////// Acelerometer
		float roll=mXfk(0,0);
		float pitch=mXfk(1,0);
		float yaw=mXfk(2,0);
		mJh.setIdentity();
		/////////////////// accelerometer
		// Fila 1
		mJh(0,0)=0;
		mJh(0,1)=-g*cos(pitch);
		mJh(0,2)=0;
		mJh(0,3)=0;
		mJh(0,4)=0;
		mJh(0,5)=0;
		mJh(0,6)=0;
		mJh(0,7)=0;
		mJh(0,8)=0;
		// Fila 2
		mJh(1,0)=g*cos(pitch)*cos(roll);
		mJh(1,1)=-g*sin(pitch)*sin(roll);
		mJh(1,2)=0;	
		mJh(1,3)=0;
		mJh(1,4)=0;
		mJh(1,5)=0;
		mJh(1,6)=0;
		mJh(1,7)=0;
		mJh(1,8)=0;
		// Fila 3
		mJh(2,0)=-g*cos(pitch)*sin(roll);
		mJh(2,1)=-g*cos(roll)*sin(pitch);
		mJh(2,2)=0;
		mJh(2,3)=0;
		mJh(2,4)=0;
		mJh(2,5)=0;
		mJh(2,6)=0;
		mJh(2,7)=0;
		mJh(2,8)=0;
		/////////////////// giroscope
		float v_roll=mXfk(3,0);	
		float v_pitch=mXfk(4,0);
		float v_yaw=mXfk(5,0);
		// Fila 4
		mJh(3,0)=0;
		mJh(3,1)=-v_yaw*cos(pitch)-v_roll*cos(yaw)*sin(pitch)-v_pitch*sin(pitch)*sin(yaw);
		mJh(3,2)=v_pitch*cos(pitch)*cos(yaw)-v_roll*cos(pitch)*sin(yaw);
		mJh(3,3)=cos(pitch)*cos(yaw);
		mJh(3,4)=cos(pitch)*sin(yaw);;
		mJh(3,5)=-sin(pitch);
		mJh(3,6)=0;
		mJh(3,7)=0;
		mJh(3,8)=0;	
		// Fila 5
		mJh(4,0)=-v_pitch*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw))+v_roll*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch))+v_yaw*cos(pitch)*cos(roll);
		mJh(4,1)=-v_yaw*sin(pitch)*sin(roll)+v_roll*cos(pitch)*cos(yaw)*sin(roll)+v_pitch*cos(pitch)*sin(roll)*sin(yaw);
		mJh(4,2)=-v_pitch*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll))-v_roll*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw));
		mJh(4,3)=-cos(roll)*sin(yaw)+cos(yaw)*sin(pitch)*sin(roll);
		mJh(4,4)=cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw);
		mJh(4,5)=cos(pitch)*sin(roll);
		mJh(4,6)=0;
		mJh(4,7)=0;
		mJh(4,8)=0;
		// Fila 6
		mJh(5,0)=-v_pitch*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw))+v_roll*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll))-v_yaw*cos(pitch)*sin(roll);
		mJh(5,1)=-v_yaw*cos(roll)*sin(pitch)+v_roll*cos(pitch)*cos(roll)*cos(yaw)+v_pitch*cos(pitch)*cos(roll)*sin(yaw);
		mJh(5,2)=v_pitch*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch))+v_roll*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw));
		mJh(5,3)=sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch);
		mJh(5,4)=-cos(yaw)*sin(roll)+cos(roll)*sin(pitch)*sin(yaw);
		mJh(5,5)=cos(pitch)*cos(roll);
		mJh(5,6)=0;
		mJh(5,7)=0;
		mJh(5,8)=0;
		/////////////////// Magnometer
		float mag=0.7;
		//// Fila 7
		mJh(6,0)=0;
		mJh(6,1)=-mag*cos(yaw)*sin(pitch);
		mJh(6,2)=-mag*cos(pitch)*sin(yaw);
		mJh(6,3)=0;
		mJh(6,4)=0;
		mJh(6,5)=0;
		mJh(6,6)=0;
		mJh(6,7)=0;
		mJh(6,8)=0;
		////// Fila 8
		//mJh(7,0)=mag*(sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch));
		//mJh(7,1)=mag*cos(pitch)*cos(yaw)*sin(roll);
		//mJh(7,2)=-mag*(cos(roll)*cos(yaw)+sin(pitch)*sin(roll)*sin(yaw));
		//mJh(7,3)=0;
		//mJh(7,4)=0;
		//mJh(7,5)=0;// (ac_roll, ac_pitch, ac_yaw)
		//mJh(7,6)=0;
		//mJh(7,7)=0;
		//mJh(7,8)=0;
		////// Fila 9
		//mJh(8,0)=mag*(cos(roll)*sin(yaw)-cos(yaw)*sin(pitch)*sin(roll));
		//mJh(8,1)=mag*cos(pitch)*cos(roll)*cos(yaw);
		//mJh(8,2)=mag*(cos(yaw)*sin(roll)-cos(roll)*sin(pitch)*sin(yaw));
		//mJh(8,3)=0;
		//mJh(8,4)=0;
		//mJh(8,5)=0;
		//mJh(8,6)=0;
		//mJh(8,7)=0;
		//mJh(8,8)=0;
	}
};

int main(int _argc,char **_argv)
{
	Eigen::Matrix<float, 9, 9> mQ; // State covariance
	mQ.setIdentity();
	mQ.block<3, 3>(0,0) *= 0.01;
	mQ.block<3, 3>(3,3) *= 0.01;
	mQ.block<3, 3>(6,6) *= 0.01;
	
	Eigen::Matrix<float, 7, 7> mR; // Observation covariance
	mR.setIdentity();
	mR.block<3, 3>(0, 0) *= 0.05;
	mR.block<3, 3>(3, 3) *= 0.05;
	mR.block<1, 1>(6, 6) *= 0.05;
	
	
	Eigen::Matrix<float, 9, 1> x0; // condiciones iniciales
	x0 << 0, 0, 0,  // (roll, pitch, yaw)
		   0, 0, 0,	// (v_roll, v_pitch, v_yaw)
		   0,0,0; 	// (ac_roll, ac_pitch, ac_yaw)
			//-0.179013878107,-1.18209290504, 9.6906709671;

	EkfEuler ekf;
	ekf.setUpEKF(mQ, mR, x0);


    ///////////////////// declaracion gráficas
	rgbd::Graph2d data_plot("ROLL,PITCH,YAW");
	std::vector<double> ROLL, PITCH, YAW;
	ROLL.push_back(0);
	PITCH.push_back(0);
	YAW.push_back(0.9);
    

	mSerialPort = new serial::Serial("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
    for (size_t i = 0; i < 20; i++)
    {
        serial_listen();
    }
    

	if(mSerialPort->isOpen()){
		std::cout << "Serial Port open!" << std::endl;
		while(true){
            
			serial_listen();

            //mtx_com.lock();
            // Normalizar magnetometro
		    float norma_mag=sqrt(xmag_new*xmag_new+ymag_new*ymag_new+zmag_new*zmag_new);
		    float xmag_norm=xmag_new/norma_mag;
		    float ymag_norm=ymag_new/norma_mag;
		    float zmag_norm=zmag_new/norma_mag;
            //mtx_com.unlock();

            Eigen::Matrix<float, 7, 1> z; // New observation
		    z << ax_new,
		    	ay_new,
		    	az_new,
		    	wi_new,
		    	wj_new,
		    	wk_new,
		    	xmag_new;
		    ekf.stepEKF(z, 0.1);

            Eigen::Matrix<float, 9, 1> filteredX = ekf.state();
		    std::cout << "magnetometro normalizado en X"  << xmag_norm << std::endl;
		    std::cout << "magnetometro normalizado en Y"  << ymag_norm << std::endl;
		    std::cout << "magnetometro normalizado en Z"  << zmag_norm << std::endl;

            if(std::isnan(filteredX[0]) || std::isnan(filteredX[1] )|| std::isnan(filteredX[2] )|| std::isnan(filteredX[3]) ) {
			std::cout << "State contains nan, ending" << std::endl;
			break;
		    }

            //////// Avisos sobre el filtro
            if (filteredX[0] > PI){
				std::cout << "El filtro no está funcionando adecuadamente derivación en ROLL" << std::endl;
			}
			if(filteredX[0] <-PI){
				std::cout << "El filtro no está funcionando adecuadamente derivación en ROLL" << std::endl;
			}
			if (filteredX[1]  > PI){
			    std::cout << "El filtro no está funcionando adecuadamente derivación en PITCH" << std::endl;
			}
			if(filteredX[1] <-PI){
				std::cout << "El filtro no está funcionando adecuadamente derivación en PITCH" << std::endl;
			}
			if (filteredX[2]  > PI){
			    std::cout << "El filtro no está funcionando adecuadamente derivación en YAW" << std::endl;
			}
			if(filteredX[2] <-PI){
				std::cout << "El filtro no está funcionando adecuadamente derivación en YAW" << std::endl;
			}

            ///////// Visualizacion
            ROLL.push_back(filteredX[0]);
		    PITCH.push_back(filteredX[1]);
		    YAW.push_back(filteredX[2]);

		    if(ROLL.size()>100)

			    ROLL.erase(ROLL.begin());

		    if(PITCH.size()>100)

			    PITCH.erase(PITCH.begin());

		    if(YAW.size()>100)

			    YAW.erase(YAW.begin());
            
            data_plot.clean();
		    data_plot.draw(ROLL, 255,0,0, rgbd::Graph2d::eDrawType::Lines);
		    data_plot.draw(PITCH, 0,255,0, rgbd::Graph2d::eDrawType::Lines);
		    data_plot.draw(YAW, 0,0,255, rgbd::Graph2d::eDrawType::Lines);
            cv::waitKey(1);
        }
		
		//return true;
	}
	else{
	  std::cout << "Serial Port failed!" << std::endl;
	  //return false;
	}
	//cv::waitKey();
}

