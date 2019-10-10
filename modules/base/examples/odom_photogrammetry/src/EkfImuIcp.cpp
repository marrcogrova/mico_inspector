///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-12-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "EkfImuIcp.h"

bool EkfImuIcp::init(cjson::Json _configFile){
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

	setUpEKF(mQ,mR,mX0);

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
double sign(double _var) {
	return _var < 0? -1:1;
}

void EkfImuIcp::updateJf(const double _incT) {
	mJf = Eigen::MatrixXd::Identity(mJf.rows(), mJf.cols());
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	mJf.block<3, 3>(0, 3) =I*_incT;
	mJf.block<3, 3>(0, 6) =I*_incT*_incT/2;
	mJf.block<3, 3>(3, 6) =I*_incT;
	
	Eigen::Vector3d auxAcc({sign(mXak(6,0)*mScaleFactor[0]), sign(mXak(6,0)*mScaleFactor[1]), sign(mXak(6,0)*mScaleFactor[2])});
	Eigen::Matrix3d auxMatAcc = (auxAcc*Eigen::Matrix<double,1,3>({1,1,1})).cwiseProduct(I);;
	mJf.block<3, 3>(6, 6) += auxMatAcc;

	mJf.block<3, 3>(6, 9) =-I;
	
	Eigen::Matrix3d diagT = (mT*Eigen::Matrix<double,1,3>({1,1,1})).cwiseProduct(I);;
	Eigen::Matrix3d diagTpluxTime = (diagT + I*_incT);
	mJf(9, 9) = diagT(0,0)/diagTpluxTime(0,0);
	mJf(10, 10) = diagT(1,1)/diagTpluxTime(1,1);
	mJf(11, 11) = diagT(2,2)/diagTpluxTime(2,2);
}

//---------------------------------------------------------------------------------------------------------------------
void EkfImuIcp::updateHZk() {
	mHZk.block<3,1>(0,0) = mXfk.block<3,1>(0,0);
	mHZk.block<3,1>(3,0) = mXfk.block<3,1>(6,0);
}

//---------------------------------------------------------------------------------------------------------------------
void EkfImuIcp::updateJh() {
	mJh(0,0) = 1;
	mJh(1,1) = 1;
	mJh(2,2) = 1;
	mJh(3,6) = 1;
	mJh(4,7) = 1;
	mJh(5,8) = 1;
}

//---------------------------------------------------------------------------------------------------------------------
// Function to convert std::string data to std::vector<double>
void string2vectord(std::string input, std::vector<double> &output){
	std::stringstream iss( input );
	double data;
	while (iss >> data){
    	output.push_back(data);
		if (iss.peek() == ',' || iss.peek() == ' ')
			iss.ignore();
	}
}

//---------------------------------------------------------------------------------------------------------------------
void EkfImuIcp::scaleFactorIMU(std::string _scaleFactor){
	std::vector<double> array;
	string2vectord(_scaleFactor,array);	
	mScaleFactor=Eigen::Vector3d(array[0],array[1],array[2]);
}

void EkfImuIcp::parameter_C1(std::string _paramC1){
	std::vector<double> array;
	string2vectord(_paramC1,array);	
	mC1=Eigen::Vector3d(array[0],array[1],array[2]);
}

void EkfImuIcp::parameter_C2(std::string _paramC2){
	std::vector<double> array;
	string2vectord(_paramC2,array);
	mC2=Eigen::Vector3d(array[0],array[1],array[2]);
}

void EkfImuIcp::parameter_T(std::string _paramT){
	std::vector<double> array;
	string2vectord(_paramT,array);
	mT=Eigen::Vector3d(array[0],array[1],array[2]);
}

void EkfImuIcp::systemCovariance(std::string _Qr0, std::string _Qr1, std::string _Qr2, std::string _Qr3, std::string _Qr4, std::string _Qr5, 
								 std::string _Qr6, std::string _Qr7, std::string _Qr8, std::string _Qr9, std::string _Qr10,std::string _Qr11){
	std::string Qr =  _Qr0 + " " + _Qr1 + " " + _Qr2 + " " + _Qr3 + " " + _Qr4 + " " + _Qr5 + " " + 
					  _Qr6 + " " + _Qr7 + " " + _Qr8 + " " + _Qr9 + " " + _Qr10 + " " + _Qr11;
	std::vector<double> array_Q;
	string2vectord(Qr,array_Q);
	mQ=Eigen::Map<Eigen::Matrix<double,12,12, Eigen::RowMajor>>(array_Q.data());
}

void EkfImuIcp::observationCovariance(std::string _Rr0,std::string _Rr1,std::string _Rr2,std::string _Rr3,std::string _Rr4,std::string _Rr5){
	std::string Rr =  _Rr0 + " " + _Rr1 + " " + _Rr2 + " " + _Rr3 + " " + _Rr4 + " " + _Rr5;
	std::vector<double> array_R;
	string2vectord(Rr,array_R);
	mR=Eigen::Map<Eigen::Matrix<double,6,6, Eigen::RowMajor>>(array_R.data());
}

void EkfImuIcp::initialState(std::string _p0, std::string _v0, std::string _a0, std::string _b0){
	std::vector<double> arrayPos,arrayVel,arrayAcc,arrayBias;
	string2vectord(_p0,arrayPos); string2vectord(_v0,arrayVel);
	string2vectord(_a0,arrayAcc); string2vectord(_b0,arrayBias);
	
	mX0.block<3,1>(0,0)=Eigen::Vector3d(arrayPos[0], arrayPos[1], arrayPos[2]);
	mX0.block<3,1>(3,0)=Eigen::Vector3d(arrayVel[0], arrayVel[1], arrayVel[2]);
	mX0.block<3,1>(6,0)=Eigen::Vector3d(arrayAcc[0], arrayAcc[1], arrayAcc[2]);
	mX0.block<3,1>(9,0)=Eigen::Vector3d(arrayBias[0],arrayBias[1], arrayBias[2]);
}
