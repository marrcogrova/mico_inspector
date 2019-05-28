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
    std::cout << "tamaño del vector:" << size << std::endl;
    std::cout << "Pos 1:" << observacion[0] << std::endl;
    std::cout << "Pos 2:" << observacion[1] << std::endl;
    std::cout << "Pos 3:" << observacion[2] << std::endl;
    std::cout << "Pos 4:" << observacion[3] << std::endl;
    std::cout << "Pos 5:" << observacion[4] << std::endl;
    std::cout << "Pos 6:" << observacion[5] << std::endl;
    std::cout << "Pos 7:" << observacion[6] << std::endl;
    std::cout << "Pos 8:" << observacion[7] << std::endl;
    std::cout << "Pos 9:" << observacion[8] << std::endl;
    std::cout << "Pos 10:" << observacion[9] << std::endl;

}
int main(int _argc,char **_argv)
{
    mSerialPort = new serial::Serial("/dev/ttyACM1", 115200, serial::Timeout::simpleTimeout(1000));
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