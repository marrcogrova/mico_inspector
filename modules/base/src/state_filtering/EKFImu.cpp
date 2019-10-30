//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Marco A. Montes Grova (a.k.a. marrcogrova) marrcogrova@gmail.com
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


#include <mico/base/state_filtering/EKFImu.h>

namespace mico{

	//---------------------------------------------------------------------------------------------------------------------
	bool EKFImu::init(cjson::Json _configFile){
		scaleFactorIMU( _configFile["scaleFactorImu"] );
		parameter_C1( _configFile["paramC1"] );
		parameter_C2( _configFile["paramC2"] );
		parameter_T( _configFile["paramT"] );
		systemCovariance( _configFile["systemCov_Q"]["row0"], _configFile["systemCov_Q"]["row1"], _configFile["systemCov_Q"]["row2"],
						_configFile["systemCov_Q"]["row3"], _configFile["systemCov_Q"]["row4"], _configFile["systemCov_Q"]["row5"],
						_configFile["systemCov_Q"]["row6"], _configFile["systemCov_Q"]["row7"], _configFile["systemCov_Q"]["row8"],
						_configFile["systemCov_Q"]["row9"], _configFile["systemCov_Q"]["row10"], _configFile["systemCov_Q"]["row11"]);
		observationCovariance( _configFile["observationCov_R"]["row0"], _configFile["observationCov_R"]["row1"], _configFile["observationCov_R"]["row2"],
							_configFile["observationCov_R"]["row3"], _configFile["observationCov_R"]["row4"], _configFile["observationCov_R"]["row5"] );
		initialState( _configFile["initState_x0"]["pos0"], _configFile["initState_x0"]["vel0"],
					_configFile["initState_x0"]["acc0"], _configFile["initState_x0"]["bias0"]);

		setUpEKF(Q_,R_,X0_);

    	return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	double sign2(double _var) {
		return _var < 0? -1:1;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::updateJf(const double _incT) {
		mJf = Eigen::MatrixXd::Identity(mJf.rows(), mJf.cols());
		Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
		mJf.block<3, 3>(0, 3) =I*_incT;
		mJf.block<3, 3>(0, 6) =I*_incT*_incT/2;
		mJf.block<3, 3>(3, 6) =I*_incT;
		
		Eigen::Vector3d auxAcc({sign2(mXak(6,0)*scaleFactor_[0]), sign2(mXak(6,0)*scaleFactor_[1]), sign2(mXak(6,0)*scaleFactor_[2])});
		Eigen::Matrix3d auxMatAcc = (auxAcc*Eigen::Matrix<double,1,3>({1,1,1})).cwiseProduct(I);;
		mJf.block<3, 3>(6, 6) += auxMatAcc;

		mJf.block<3, 3>(6, 9) =-I;
		
		Eigen::Matrix3d diagT = (T_*Eigen::Matrix<double,1,3>({1,1,1})).cwiseProduct(I);;
		Eigen::Matrix3d diagTpluxTime = (diagT + I*_incT);
		mJf(9, 9) = diagT(0,0)/diagTpluxTime(0,0);
		mJf(10, 10) = diagT(1,1)/diagTpluxTime(1,1);
		mJf(11, 11) = diagT(2,2)/diagTpluxTime(2,2);
	}

	//---------------------------------------------------------------------------------------------------------------------
	Eigen::Matrix<double,6,1> EKFImu::observationFunction(Eigen::Matrix<double,12,1> _Xfk){
		Eigen::Matrix<double,6,1> H;
		H.block<3,1>(0,0) = _Xfk.block<3,1>(0,0);
		H.block<3,1>(3,0) = _Xfk.block<3,1>(6,0);

		return H;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::updateHZk() {
		mHZk = observationFunction(mXfk);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::updateJh() {
		mJh(0,0) = 1;
		mJh(1,1) = 1;
		mJh(2,2) = 1;
		mJh(3,6) = 1;
		mJh(4,7) = 1;
		mJh(5,8) = 1;
	}

	//---------------------------------------------------------------------------------------------------------------------
	// Function to convert std::string data to std::vector<>
	template <typename _dataType>
	void string2vector(std::string input, std::vector<_dataType> &output){
		std::stringstream iss( input );
		_dataType data;
		while (iss >> data){
			output.push_back(data);
			if (iss.peek() == ',' || iss.peek() == ' ')
				iss.ignore();
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::scaleFactorIMU(std::string _scaleFactor){
		std::vector<double> array;
		string2vector<double>(_scaleFactor,array);	
		scaleFactor_=Eigen::Vector3d(array[0],array[1],array[2]);
	}
	
	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::parameter_C1(std::string _paramC1){
		std::vector<double> array;
		string2vector<double>(_paramC1,array);	
		C1_=Eigen::Vector3d(array[0],array[1],array[2]);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::parameter_C2(std::string _paramC2){
		std::vector<double> array;
		string2vector<double>(_paramC2,array);
		C2_=Eigen::Vector3d(array[0],array[1],array[2]);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::parameter_T(std::string _paramT){
		std::vector<double> array;
		string2vector<double>(_paramT,array);
		T_=Eigen::Vector3d(array[0],array[1],array[2]);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::systemCovariance(std::string _Qr0, std::string _Qr1, std::string _Qr2, std::string _Qr3, std::string _Qr4, std::string _Qr5, 
									std::string _Qr6, std::string _Qr7, std::string _Qr8, std::string _Qr9, std::string _Qr10,std::string _Qr11){
		std::string Qr =  _Qr0 + " " + _Qr1 + " " + _Qr2 + " " + _Qr3 + " " + _Qr4 + " " + _Qr5 + " " + 
						_Qr6 + " " + _Qr7 + " " + _Qr8 + " " + _Qr9 + " " + _Qr10 + " " + _Qr11;
		std::vector<double> array_Q;
		string2vector<double>(Qr,array_Q);
		Q_=Eigen::Map<Eigen::Matrix<double,12,12, Eigen::RowMajor>>(array_Q.data());
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::observationCovariance(std::string _Rr0,std::string _Rr1,std::string _Rr2,std::string _Rr3,std::string _Rr4,std::string _Rr5){
		std::string Rr =  _Rr0 + " " + _Rr1 + " " + _Rr2 + " " + _Rr3 + " " + _Rr4 + " " + _Rr5;
		std::vector<double> array_R;
		string2vector<double>(Rr,array_R);
		R_=Eigen::Map<Eigen::Matrix<double,6,6, Eigen::RowMajor>>(array_R.data());
	}

	//---------------------------------------------------------------------------------------------------------------------
	void EKFImu::initialState(std::string _p0, std::string _v0, std::string _a0, std::string _b0){
		std::vector<double> arrayPos,arrayVel,arrayAcc,arrayBias;
		string2vector<double>(_p0,arrayPos); string2vector<double>(_v0,arrayVel);
		string2vector<double>(_a0,arrayAcc); string2vector<double>(_b0,arrayBias);
		
		X0_.block<3,1>(0,0)=Eigen::Vector3d(arrayPos[0], arrayPos[1], arrayPos[2]);
		X0_.block<3,1>(3,0)=Eigen::Vector3d(arrayVel[0], arrayVel[1], arrayVel[2]);
		X0_.block<3,1>(6,0)=Eigen::Vector3d(arrayAcc[0], arrayAcc[1], arrayAcc[2]);
		X0_.block<3,1>(9,0)=Eigen::Vector3d(arrayBias[0],arrayBias[1], arrayBias[2]);
	}

}
