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
        /** \brief Set observations
        */
        void observations(std::vector<Eigen::Matrix<float,Dim_, 1>> &_observations);

        /** \brief Set initial gaussians
        */
        void initGaussians(std::vector<GaussianParameters<Dim_>> &_gaussians);

		/** \brief Iterate. 777 need review
		*/
		bool iterate();

		/** \brief get result of iterations
		*/
        std::vector<GaussianParameters<Dim_>> result() const { return mGaussians; }

	private:
        std::vector<Eigen::Matrix<float, Dim_, 1>> mObservations;
        std::vector<GaussianParameters<Dim_>> mGaussians;
	};
}	//

#include "GMMEM.inl"

#endif	//	RGBDTOOLS_MAP3D_GMMEM_H_
