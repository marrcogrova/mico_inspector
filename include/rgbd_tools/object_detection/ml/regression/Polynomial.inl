//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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

namespace rgbd_tools{
	//-------------------------------------------------------------------------------------------------------------------
	template<unsigned Nvars_, unsigned Nmonomials_>
	Polynomial<Nvars_, Nmonomials_>::Polynomial(std::function<Monomials(const Input &)> _monomials): mMonomialCalculator(_monomials) {
	}

	//-------------------------------------------------------------------------------------------------------------------
	template<unsigned Nvars_, unsigned Nmonomials_>
	void Polynomial<Nvars_, Nmonomials_>::setParams(const Params &_params) {
		mParams = _params;
	}

	//-------------------------------------------------------------------------------------------------------------------
	template<unsigned Nvars_, unsigned Nmonomials_>
	double Polynomial<Nvars_, Nmonomials_>::evaluate(const Input &_x) const{
		Monomials monom = mMonomialCalculator(_x);
		return mParams*monom;
	}

	//-------------------------------------------------------------------------------------------------------------------
	template<unsigned Nvars_, unsigned Nmonomials_>
	Eigen::Matrix<double, 1, Nmonomials_> Polynomial<Nvars_, Nmonomials_>::parameters() const {
		return mParams;
	}

	//-------------------------------------------------------------------------------------------------------------------
	template<unsigned Nvars_, unsigned Nmonomials_>		
	std::function<Eigen::Matrix<double, Nmonomials_, 1>(const Eigen::Matrix<double, 1, Nvars_> &)> Polynomial<Nvars_, Nmonomials_>::monomialCalculator() const {
		return mMonomialCalculator;
	}
}	//	namespace rgbd_tools