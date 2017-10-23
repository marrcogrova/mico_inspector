///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		EC-SAFEMOBIL: 
//			Author: Pablo Ramon Soria
//			Date:	2015-FEB-10
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Gaussian Mixture Model - Expectation-Maximizacion algorithm.

#include <Eigen/Eigen>
#include <iostream>


using namespace std;

namespace rgbd{

	double gaussianMixtureModel(const Eigen::Vector2d& _nu, const Eigen::Matrix2d& _cov, const Eigen::Vector2d& _x);

	//---------------------------------------------------------------------------------------------------------------------
    GMMEM::GMMEM(std::vector<Eigen::Matrix<float,Dim_, 1>> &_observations, std::vector<GaussianParameters> &_gaussians): mObservations(_observations), mGaussians(_gaussians){

    }

	//---------------------------------------------------------------------------------------------------------------------
	bool GMMEM::iterate(){
		Eigen::MatrixXd membershipWeighs(mGaussians.size(), mParticles.size());
		Eigen::VectorXd N(mGaussians.size());

		if (mGaussians.size() == 0)
			return false;	// No gaussians to iterate.

		unsigned steps = 0;
		double lastLikelihood = 0.0;
		double errorLikelihood = 0.0;
		do{
			double likelihood = 0.0;
			// Expectation step  ----------------
			// Calculate wik matrix dim(wik) = {n clusters, n particles}
			//		wik = gaussian(nuk, covk, xi)*alphai / (sum[m=1:k](gaussian(num, covm, xi)*alpham))
			for (unsigned pIndex = 0; pIndex < mParticles.size(); pIndex++){
                double den = 0.0;
				for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                    den += gaussianMixtureModel(mGaussians[gIndex].mean,
                                                mGaussians[gIndex].covariance,
                                                mParticles[pIndex]) * mGaussians[gIndex].mixtureWeigh;
				}

				for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                    membershipWeighs(gIndex, pIndex) = gaussianMixtureModel(mGaussians[gIndex].mean,
                                                                            mGaussians[gIndex].covariance,
                                                                            mParticles[pIndex]) * mGaussians[gIndex].mixtureWeigh / den;
				}
			}
			/*std::cout << "--------------------------------" << std::endl;
			std::cout << "Membership Weighs: " << std::endl;
			std::cout << membershipWeighs.transpose() << std::endl;*/

			// Maximization step ----------------
			// Calculate: Nk = sum[i=1:N](wik)	; N = n particles
			for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
				N[gIndex] = 0.0;
				for (unsigned pIndex = 0; pIndex < mParticles.size(); pIndex++){
					N[gIndex] += membershipWeighs(gIndex, pIndex);
				}
			}
			/*std::cout << "--------------------------------" << std::endl;
			std::cout << "N: " << std::endl;
			std::cout << N << std::endl;*/
			// Calculate mixture weighs: alpha = Nk/N
			for (unsigned i = 0; i < mGaussians.size(); i++){
				mGaussians[i].mixtureWeigh = N[i] / mParticles.size();
			}

			// Calculate: nuk = (1/Nk) * sum[i=1:N](wik*Xi)
			for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                Eigen::Matrix<float, Dim_, 1> mean = Eigen::Matrix<float, Dim_, 1>::Zeros();
				for (unsigned pIndex = 0; pIndex < mParticles.size(); pIndex++){
					Eigen::Vector2d particle;
					particle << mParticles[pIndex][0], mParticles[pIndex][1];
					mean += membershipWeighs(gIndex, pIndex)*particle;
				}
				mean /= N[gIndex];
                mGaussians[gIndex].mean = mean;
			}

			// Calculate: Covk = (1/Nk) * sum[i=1:N](wik * (xi - nuk)(xi - nuk).transpose()
			for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
                auto mean = mGaussians[gIndex].mean;
                Eigen::Matrix<float, Dim_, Dim_> cov = Eigen::Matrix<float, Dim_, Dim_>::Zeros();
				for (unsigned pIndex = 0; pIndex < mParticles.size(); pIndex++){
					Eigen::Vector2d particle;
					particle << mParticles[pIndex][0], mParticles[pIndex][1];
					cov += membershipWeighs(gIndex, pIndex)*(particle - mean)*(particle - mean).transpose();
				}
				cov /= N[gIndex];
                mGaussians[gIndex].covariance = cov;

				/*std::cout << "--------------------------------" << std::endl;
				std::cout << "cov " << gIndex << ": " << std::endl;
				std::cout << cov << std::endl;*/
			}

			// Calc of log-likelihood
			for (unsigned pIndex = 0; pIndex < mParticles.size(); pIndex++){
				double gProb = 0.0;
				Eigen::Vector2d particle;
				particle << mParticles[pIndex][0], mParticles[pIndex][1];
				for (unsigned gIndex = 0; gIndex < mGaussians.size(); gIndex++){
					Eigen::Vector2d mean;
					mean << mGaussians[gIndex].mean[0], mGaussians[gIndex].mean[1];
					Eigen::Matrix2d cov;
					cov << mGaussians[gIndex].covariance[0], mGaussians[gIndex].covariance[1], mGaussians[gIndex].covariance[2], mGaussians[gIndex].covariance[3];
					gProb += gaussianMixtureModel(mean, cov, particle) * mGaussians[gIndex].mixtureWeigh;
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
	double gaussianMixtureModel(const Eigen::Vector2d& _nu, const Eigen::Matrix2d& _cov, const Eigen::Vector2d& _x){
		double covDet = _cov.determinant();
        // Probability distribution function.

		double res = 1 / (sqrt(pow(2 * M_PI, 2)*covDet));
		res *= exp(-0.5* (_x - _nu).transpose()*_cov.inverse()*(_x - _nu));
		return res;
	}
}	//	namespace BOViL
