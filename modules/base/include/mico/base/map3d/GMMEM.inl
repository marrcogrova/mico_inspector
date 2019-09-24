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


#include <Eigen/Eigen>
#include <iostream>


using namespace std;

namespace mico {
    template<int Dim_>
    double gaussianMixtureModel(const Eigen::Matrix<float,Dim_,1>& _nu, const Eigen::Matrix<float, Dim_, Dim_>& _cov, const Eigen::Matrix<float, Dim_, 1>& _x){
        double covDet = _cov.determinant();
        // Probability distribution function.

        double res = 1 / (sqrt(pow(2 * M_PI, 2)*covDet));
        res *= exp(-0.5* (_x - _nu).transpose()*_cov.inverse()*(_x - _nu));
        return res;
    }


	//---------------------------------------------------------------------------------------------------------------------
    template<int Dim_>
    bool GMMEM<Dim_>::iterate(){
        Eigen::MatrixXd membershipWeighs(mGaussians.size(), mObservations.size());
		Eigen::VectorXd N(mGaussians.size());

        if (mGaussians.size() == 0){
            std::cout << "Not given any initial gaussian, not doing GMMEM." << std::endl;
			return false;	// No gaussians to iterate.
        }
		unsigned steps = 0;
		double lastLikelihood = 0.0;
		double errorLikelihood = 0.0;
		do{
			double likelihood = 0.0;
			// Expectation step  ----------------
			// Calculate wik matrix dim(wik) = {n clusters, n particles}
			//		wik = gaussian(nuk, covk, xi)*alphai / (sum[m=1:k](gaussian(num, covm, xi)*alpham))
            for (unsigned pIndex = 0; pIndex < mObservations.size(); pIndex++){
                double den = 0.0;
				for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                    den += gaussianMixtureModel<Dim_>(mGaussians[gIndex].mean,
                                                mGaussians[gIndex].covariance,
                                                mObservations[pIndex]) * mGaussians[gIndex].mixtureWeigh;
				}

				for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                    membershipWeighs(gIndex, pIndex) = gaussianMixtureModel<Dim_>(mGaussians[gIndex].mean,
                                                                            mGaussians[gIndex].covariance,
                                                                            mObservations[pIndex]) * mGaussians[gIndex].mixtureWeigh / den;
				}
			}
			/*std::cout << "--------------------------------" << std::endl;
			std::cout << "Membership Weighs: " << std::endl;
			std::cout << membershipWeighs.transpose() << std::endl;*/

			// Maximization step ----------------
			// Calculate: Nk = sum[i=1:N](wik)	; N = n particles
			for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
				N[gIndex] = 0.0;
                for (unsigned pIndex = 0; pIndex < mObservations.size(); pIndex++){
					N[gIndex] += membershipWeighs(gIndex, pIndex);
				}
			}
			/*std::cout << "--------------------------------" << std::endl;
			std::cout << "N: " << std::endl;
			std::cout << N << std::endl;*/
			// Calculate mixture weighs: alpha = Nk/N
			for (unsigned i = 0; i < mGaussians.size(); i++){
                mGaussians[i].mixtureWeigh = N[i] / mObservations.size();
			}

			// Calculate: nuk = (1/Nk) * sum[i=1:N](wik*Xi)
			for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                Eigen::Matrix<float, Dim_, 1> mean = Eigen::Matrix<float, Dim_, 1>::Zero();
                for (unsigned pIndex = 0; pIndex < mObservations.size(); pIndex++){
                    mean += membershipWeighs(gIndex, pIndex)*mObservations[pIndex];
				}
				mean /= N[gIndex];
                mGaussians[gIndex].mean = mean;
			}

			// Calculate: Covk = (1/Nk) * sum[i=1:N](wik * (xi - nuk)(xi - nuk).transpose()
			for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                auto mean = mGaussians[gIndex].mean;
                Eigen::Matrix<float, Dim_, Dim_> cov = Eigen::Matrix<float, Dim_, Dim_>::Zero();
                for (unsigned pIndex = 0; pIndex < mObservations.size(); pIndex++){
                    cov += membershipWeighs(gIndex, pIndex)*(mObservations[pIndex]- mean)*(mObservations[pIndex]- mean).transpose();
				}
				cov /= N[gIndex];
                mGaussians[gIndex].covariance = cov;

				/*std::cout << "--------------------------------" << std::endl;
				std::cout << "cov " << gIndex << ": " << std::endl;
				std::cout << cov << std::endl;*/
			}

			// Calc of log-likelihood
            for (unsigned pIndex = 0; pIndex < mObservations.size(); pIndex++){
                double gProb = 0.0;
				for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                    gProb += gaussianMixtureModel<Dim_>(mGaussians[gIndex].mean,
                                                  mGaussians[gIndex].covariance,
                                                  mObservations[pIndex])
                                        * mGaussians[gIndex].mixtureWeigh;
				}
				likelihood += log(gProb);
			}

			if (std::isnan(likelihood) || std::isinf(likelihood)){	// Error converging to solution.
				return false;
			}

			errorLikelihood = abs(lastLikelihood - likelihood);
			std::cout << "lastLL: " << lastLikelihood << " - CurrentLL:" << likelihood << ". Error: " << errorLikelihood << std::endl;
			lastLikelihood = likelihood;
			steps++;
		} while (steps < 10 && errorLikelihood > 1);

		return true;
	}

    //---------------------------------------------------------------------------------------------------------------------
    template<int Dim_>
    void GMMEM<Dim_>::observations(std::vector<Eigen::Matrix<float, Dim_, 1> > &_observations) {
        mObservations= _observations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<int Dim_>
    void GMMEM<Dim_>::initGaussians(std::vector<GaussianParameters<Dim_>> &_gaussians) {
        mGaussians = _gaussians;
    }

}	//	
