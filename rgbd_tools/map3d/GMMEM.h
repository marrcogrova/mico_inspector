///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		EC-SAFEMOBIL: 
//			Author: Pablo Ramon Soria
//			Date:	2015-FEB-10
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Gaussian Mixture Model - Expectation-Maximizacion algorithm.


#ifndef RGBDTOOLS_MAP3D_GMMEM_H_
#define RGBDTOOLS_MAP3D_GMMEM_H_

#include <array>	// 666 TODO solve.
#include <vector>

namespace rgbd {
	/** Struct that holds parameters of a gaussian distribution for a Gaussian Mixture Model (GMM)
	*/
    template<int Dim_>
	struct GaussianParameters{
		GaussianParameters(){};
        GaussianParameters(float _mixtureWeigh, Eigen::Matrix<float, Dim_, 1> &_mean, Eigen::Matrix<float, Dim_, Dim_> &_covariance) :
            mixtureWeigh(_mixtureWeigh),
            mean(_mean),
            covariance(_covariance) {};

        float                               mixtureWeigh;
        Eigen::Matrix<float, Dim_, 1>       mean;
        Eigen::Matrix<float, Dim_, Dim_>    covariance;
	};


	/** Class that implements a Gaussian Mixture Model Expectation-Maximization algorithm (GMMEM). 777 add reference.
	*	Particularly, classify a disjoined set of particles in 2D plane into a set of gaussians distributions.
	*/
    template<int Dim_>
	class GMMEM{
	public:
		/** \brief Constructor. Receive initial set of gaussians to iterate with.
		*/
        GMMEM(std::vector<Eigen::Matrix<float,Dim_, 1>> &_observations, std::vector<GaussianParameters> &_gaussians);

		/** \brief Iterate. 777 need review
		*/
		bool iterate();

		/** \brief get result of iterations
		*/
		std::vector<GaussianParameters> result() const { return mGaussians; };

	private:
        std::vector<Eigen::Matrix<float, Dim_, 1>> mObservations;
		std::vector<GaussianParameters> mGaussians;
	};
}	//

#include "GMMEM.inl"

#endif	//	RGBDTOOLS_MAP3D_GMMEM_H_
