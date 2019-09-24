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

#ifndef MICO_BASE_OBJECTDETECTION_ML_REGRESSION_POLYNOMIAL_H_
#define MICO_BASE_OBJECTDETECTION_ML_REGRESSION_POLYNOMIAL_H_

#include <Eigen/Eigen>
#include <functional>

namespace mico {
	/// Base template for multivariable polynomial representation
	///	\tparam Nvar_	number of variables.
	///	\tparam Size_	number of elements / Number of parameters.
	///
	///		ex. y = a*x1 +  b*x1^2 + c*x1*x2 + d*x2^2
	///				Nvar_ = 2.
	///				Size_ = 4.
	///
	template<unsigned Nvars_, unsigned Nmonomials_>
	class Polynomial {
	public:
		/// Redefinitions to simplify usage.
		typedef Eigen::Matrix<double, 1, Nvars_>		Input;
		typedef Eigen::Matrix<double, Nmonomials_, 1>	Monomials;
		typedef Eigen::Matrix<double, 1, Nmonomials_>	Params;

		/// Polynomial Constructor. 
		/// \param _base: base form of polynomial
		///
		///		ex. y = a + b*x1 +  c*x1^2 + d*x1*x2 + e*x2^2
		///		auto base = [] (double _x1, double _x2)
		///		{
		///			Eigen::Matrix\<double, 2, 5\> baseVec;
		///			base << 1		<--- x0 = 1 always.
		///					_x1,
		///					_x1^2,
		///					_x1*x2,
		///					_x2^2;
		///			return baseVec;
		///		}
		///		Polynomial\<2,5\> poly(base);
		///
		Polynomial(std::function<Monomials(const Input &)> _monomials);

		/// Define params of the polymonial equation.
		///
		///		ex. y = 1 + x1 +  -3*x1^2 + 8*x1*x2 + x2^2
		///		Eigen::Matrix\<double, 1, 5\> params;
		///		params << 1, 1, -3, 8, 1;
		///		poly.setParams(params);
		///
		void setParams(const Params &_params);

		///	Evaluate polynomial equation with the given values of variables
		///		
		///		ex. y = 1 + x1 +  -3*x1^2 + 8*x1*x2 + x2^2
		///			y(1,2) = 19;
		///		
		///		Eigen::Matrix\<double, 1, 2\> x;
		///		x << 1, 2;
		///		y = poly.evaluate(x);
		///
		double evaluate(const Input &_x) const;

		/// Get current parameter list.
		Params parameters() const;

		/// Get calculator of monomials.
		std::function<Monomials(const Input &)> monomialCalculator() const;

	private:
		std::function<Monomials(const Input &)> mMonomialCalculator;
		Params	mParams;
	};

}	//	namespace mico 

#include <mico/base/object_detection/ml/regression/Polynomial.inl>

#endif	