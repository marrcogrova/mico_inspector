//---------------------------------------------------------------------------------------------------------------------
//  mico
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


#ifndef MICO_BASE_MAP3D_GMMEM_H_
#define MICO_BASE_MAP3D_GMMEM_H_

#include <array>	// 666 TODO solve.
#include <vector>

namespace mico {
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
