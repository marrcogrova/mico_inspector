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


#ifndef MICO_BASE_OBJECTDETECTION_ML_REGRESSION_REGRESSION_H_
#define MICO_BASE_OBJECTDETECTION_ML_REGRESSION_REGRESSION_H_

#include <array>
#include <Eigen/Eigen>
#include <vector>

#include <mico/base/object_detection/ml/classification/Polynomial.h>

namespace mico {
	/// Base tamplate of Regression algorithms (i.e. LinealRegression, PolinomialRegression, Logistic Regression, 
	/// etc.). Now cost function is always square function. Regression takes as template arguments:
	/// \tparam InputSize_ number of variables + 1. (ex. a0 + a1*x1 + a2*x2 --> InputSize_ = 3)
	template<unsigned Nvars_, unsigned Nmonomials_>
	class Regression{
	public:	// Public interface
		/// Redefinition for simplyfied understanding
		typedef std::function<Eigen::Matrix<double, Nmonomials_, 1>(const Eigen::Matrix<double, 1, Nvars_> &)> Hypothesis;

		/// Build a regression with the given hypothesys.
		/// \param _hypothesis Polinomial equation that defines the hypothesis
		/// \param _transformation Additional transformation (g(x)) that can be applied to hypothesis, for example: sigmoid to use regression as logistic regression. Default g(x) = x;
		Regression(const Polynomial<Nvars_, Nmonomials_> &_hypothesis, const std::function<double(double)> &_transformation = [](double _x) {return _x;});

		/// \brief Traing network with given dataset.
		/// \tparam TrainSize_ size of training set
		/// \param _x inputs of datasets.
		/// \param _y desired results for given inputs.
		/// \param _alpha gradient coefficient
		/// \param _lambda regularization parameter
		/// \param _maxIter maximum number of iteration allowed
		/// \param _tol min difference required between steps in cost function
		template <unsigned TrainSize_>
		void train(const Eigen::Matrix<double, TrainSize_, Nvars_> &_x, const Eigen::Matrix<double, TrainSize_, 1> &_y, double _alpha, double _lambda, unsigned _maxIter = 150, double _tol = 0.00001);

		/// \brief Prediction of Regression.
		/// \param Input values.
		double evaluate(const Eigen::Matrix<double, 1, Nvars_> &_x) const;

		Polynomial<Nvars_, Nmonomials_> hypothesis() const;
	private:
		template <unsigned TrainSize_>
		Eigen::Matrix<double, TrainSize_, Nmonomials_>	adaptSet(const Eigen::Matrix<double, TrainSize_,Nvars_> &_x) const;
		Eigen::Matrix<double, Nmonomials_,1>			gradient(const Eigen::Matrix<double, 1, Nmonomials_> &_x, double _y) const;

		// For debugging purposes.
		double cost(const Eigen::Matrix<double, 1, Nmonomials_> &_x, double _y) const;

	private:	// Private members
		Polynomial<Nvars_, Nmonomials_>	mHypothesis;
		std::function<double(double)> mTransformation;
	};
}	//	namespace mico 

#include <mico/base/object_detection/ml/regression/Regression.inl>

#endif 