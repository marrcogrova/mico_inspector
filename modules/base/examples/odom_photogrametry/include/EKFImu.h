//---------------------------------------------------------------------------------------------------------------------
//  EKF IMU
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018  Pablo Ramon Soria (a.k.a. Bardo91) & Marco Montes Grova (a.k.a marrcogrova)
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

#ifndef EKFIMU_H_
#define EKFIMU_H_


#include <mico/base/state_filtering/ExtendedKalmanFilter.h>
#include <mico/base/cjson/json.h>


// Estimation of th drone position using Acc and gyro.
//
// State vector Xk = {x, y, z, vx, vy, vz, ax, ay, az, bax, bay, baz}
//
//
//
class EKFImu : public  mico::ExtendedKalmanFilter<double,12,6>{
public:
	// Inicializes EKF parameters
	virtual bool init(cjson::Json _configFile);

	/// \brief Set factor param used to adjust IMU data.
    /// \param _factor: scaleFactor.
    void scaleFactorIMU(std::string _scaleFactor);

	/// \brief Set factor param C1 used to adjust IMU bias.
    /// \param _factor: paraC1_.
    void parameter_C1(std::string _paraC1_);

	/// \brief Set factor param C2 used to adjust IMU bias.
    /// \param _factor: paraC2_.
    void parameter_C2(std::string _paraC2_);

	/// \brief Set factor param T used to adjust IMU bias.
    /// \param _factor: paramT.
    void parameter_T(std::string _paramT);

	/// \brief Set system covariance matrix.
    /// \param _factor: Q.
    void systemCovariance(std::string _Qr0, std::string _Qr1, std::string _Qr2, std::string _Qr3, std::string _Qr4, std::string _Qr5, 
						  std::string _Qr6, std::string _Qr7, std::string _Qr8, std::string _Qr9, std::string _Qr10,std::string _Qr11);

	/// \brief Set observation covariance matrix.
    /// \param _factor: R.
    void observationCovariance(std::string _Rr0,std::string _Rr1,std::string _Rr2,std::string _Rr3,std::string _Rr4,std::string _Rr5);

	/// \brief Set EKF initial state.
    /// \param _factor: x0.
    void initialState(std::string _p0, std::string _v0, std::string _a0, std::string _b0);

protected:
	void updateJf(const double _incT);
	void updateHZk();
	void updateJh();

private:
	Eigen::Vector3d scaleFactor_=Eigen::Vector3d( 1.0 , 1.0 , 1.0 ); 	// _scaleFactor (must be estimated)
	
	// Values taken from -> https://pdfs.semanticscholar.org/7e13/6039c245d21a070869779286675180ac1fba.pdf
	Eigen::Vector3d C1_ =Eigen::Vector3d( 0.7562 , 0.0514 , 1.138);		// _c1 [m/(s^2),m/(s^2),deg/s]
	Eigen::Vector3d C2_ =Eigen::Vector3d(-0.8255 , 0.0451 , 16.41);		// _c2 [m/(s^2),m/(s^2),deg/s]
	Eigen::Vector3d mT  =Eigen::Vector3d( 0.3042 , 0.7573 , 1.565);		//  _t [s, s, s]
	
	
	Eigen::Matrix<double,12,12> Q_ = Eigen::Matrix<double,12,12>::Identity();  // State covariance
  	Eigen::Matrix<double,6,6>   mR = Eigen::Matrix<double,6,6>::Identity();    // Observation covariance
  	Eigen::Matrix<double,12,1> mX0 = (Eigen::Matrix<double,12,1>() <<  // Initial state
                                            0.0,   0.0,  45.0,             // x,y,z
                                            1.0,   1.0,   1.0,             // vx,vy,vz
                                            1.0,   1.0,   1.0,             // ax,ay,az
                                          -0.14, 0.051, 6.935).finished(); // bx,by,bz
};

#endif	//	EKFIMU_H_