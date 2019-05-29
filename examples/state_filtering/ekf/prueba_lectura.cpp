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

serial::Serial  *mSerialPort = nullptr; 
 std::vector<std::string> observacion;
float wi_new=99, wj_new=99, wk_new=99;
float ax_new = 99, ay_new = 99, az_new = 99, xmag_new = 99,ymag_new=99, zmag_new=99;
std::mutex mtx_com;


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
    while(size!=10){
        std::string resultRead = mSerialPort->readline(65536, "\r\n");
        std::cout << "LECTURA PUERTO SERIE:" << resultRead << std::endl;///////////// Nos da un vector pero con los números independientes   
        split(resultRead,observacion,'\t');
        size=observacion.size();
    }

}
int main(int _argc,char **_argv)
{
    mSerialPort = new serial::Serial("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
    if(mSerialPort->isOpen()){
        std::cout << "Serial Port open!" << std::endl;
        while (true)
        {
            //std::string resultRead = mSerialPort->readline(65536, "\r\n");
            //std::cout << "LECTURA PUERTO SERIE:" << resultRead << std::endl;
            serial_listen();

          
        
        }
    }    

}
