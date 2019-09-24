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


#include <iostream>

namespace mico {
	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	void NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::train(const Eigen::MatrixXd &_x, const Eigen::MatrixXd &_y, double _alpha, double _lambda, unsigned _maxIter, double _tol) {
		//assert(_x.cols == int(InputSize_));
		//assert(_y.cols == int(OutputSize_));

		// Calculate media and sigma of input and normalize dataset
		Eigen::MatrixXd x = normalizeDataset(_x);

		// Init params randomlly to decouple params.
		randomizeParams();
		unsigned iters = 0;
		double lastCost = 999999;
		double cost = 0;
		do {
			std::array<Eigen::MatrixXd, HiddenLayers_ + 2 - 1> gradients;
			// Init gradients;
			gradients[0] = Eigen::MatrixXd::Zero(HiddenUnits_, InputSize_+1);
			for (unsigned i = 1; i < gradients.size(); i++) {
				gradients[i] = Eigen::Matrix<double, HiddenUnits_, HiddenUnits_+1>::Zero();
			}
			gradients[gradients.size()-1] = Eigen::MatrixXd::Zero(OutputSize_, HiddenUnits_+1);

			// For every set
			for (int set = 0; set < x.rows(); set++) {
				//  --- Feedforward propagation ---
				std::array<MatrixXd, HiddenLayers_ + 2> a;
				std::array<MatrixXd, HiddenLayers_ + 2> z;

				// Initial activation
				a[0] = x.block<1, InputSize_>(set, 0).transpose();
				// Activations
				for (unsigned layer = 1; layer < HiddenLayers_ + 2; layer++) {
					z[layer] = mParameters[layer-1]*appendBias(a[layer-1]);
					a[layer] = sigmoid(z[layer]);
				}
				// Compute cost
				Eigen::MatrixXd h = a[a.size()-1];
				Eigen::MatrixXd y = _y.block<1, OutputSize_>(set,0).transpose();
				Eigen::MatrixXd vOnes = Eigen::MatrixXd::Ones(OutputSize_, 1);
				cost += -( y.cwiseProduct(logarithm(h))+ (vOnes - y).cwiseProduct(logarithm(vOnes - h))).sum();

				//  --- Back propagation ---
				std::array<MatrixXd, HiddenLayers_ + 2> d;

				d[a.size()-1] = a[a.size()-1] - y;
				for (unsigned i = (HiddenLayers_ + 2 - 1) - 1; i > 0; i--) {
					d[i] = (mParameters[i].block(0, 1, mParameters[i].rows(), mParameters[i].cols()-1).transpose()*d[i+1]).cwiseProduct(sigmoidGradient(z[i]));
				}

				for (unsigned i = 0; i < HiddenLayers_ + 2 - 1; i++) {
					gradients[i] += d[i+1]*appendBias(a[i]).transpose();
				}
			}
			cost = cost/x.size();
			for (unsigned i = 0; i < HiddenLayers_ + 2 - 1; i++) {
				gradients[i] /= x.size();
			}

			// Regularize cost function.
			for (unsigned i = 0; i < HiddenLayers_ + 2-1; i++) {
				Eigen::MatrixXd aux = mParameters[i].block(0,1, mParameters[i].rows(), mParameters[i].cols()-1);
				cost += aux.cwiseProduct(aux).sum()*_lambda/2/x.size();;
			}

			// Regularize gradient.
			for (unsigned i = 0; i < HiddenLayers_ + 2-1; i++) {
				auto aux = mParameters[i];
				aux.block(0,0,mParameters[i].rows(), 1) = Eigen::MatrixXd::Zero(mParameters[i].rows(),1);
				gradients[i] += _lambda/x.size()*aux;
			}

			for (unsigned i = 0; i < HiddenLayers_ + 2-1; i++) {
				mParameters[i].block(0,1, mParameters[i].rows(), mParameters[i].cols()-1) += - _alpha*gradients[i].block(0,1, gradients[i].rows(), gradients[i].cols()-1);
			}
			mCostHistory.push_back(cost);
			iters++;
		}while(iters < _maxIter/* && abs(lastCost - cos) > _tol*/);
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	Eigen::Matrix<double, OutputSize_, 1> NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::evaluate(const Eigen::Matrix<double, InputSize_, 1> &_x) {
		// Normalize input;
		auto x = normalizeInput(_x);

		//  --- Feedforward propagation ---
		std::array<MatrixXd, HiddenLayers_ + 2> a;
		std::array<MatrixXd, HiddenLayers_ + 2> z;
		// Initial activation
		a[0] = x;
		// Activations
		for (unsigned layer = 1; layer < HiddenLayers_ + 2; layer++) {
			z[layer] = mParameters[layer-1]*appendBias(a[layer-1]);
			a[layer] = sigmoid(z[layer]);
		}

		return a[a.size()-1];
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	std::array<Eigen::MatrixXd, HiddenLayers_ + 2 -1> NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::parameters(){
		return mParameters;
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	void  NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::parameters(const std::array<Eigen::MatrixXd, HiddenLayers_ + 2 - 1> &_parameters) {
		mParameters = _parameters;
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	std::pair<Eigen::Matrix<double, 1, InputSize_>, Eigen::Matrix<double, 1, InputSize_>> NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::normalParams() {
		return mNormalizeParameters;
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	void NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::normalParams(const std::pair<Eigen::Matrix<double, 1, InputSize_>, Eigen::Matrix<double, 1, InputSize_>> &_normalParams) {
		mNormalizeParameters = _normalParams;
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	std::vector<double> NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::costHistory() {
		return mCostHistory;
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	void NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::randomizeParams() {	
		mParameters[0] = Eigen::Matrix<double, HiddenUnits_, InputSize_ + 1>::Random();
		for (unsigned i = 0; i < HiddenLayers_ - 1; i++) {
			mParameters[i+1] = Eigen::Matrix<double, HiddenUnits_, HiddenUnits_+1>::Random();
		}
		mParameters[mParameters.size()-1] = Eigen::Matrix<double, OutputSize_,HiddenUnits_ + 1> ::Random();
	}
	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	Eigen::MatrixXd NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::appendBias(const Eigen::MatrixXd &_x) {
		Eigen::MatrixXd x(_x.rows() + 1, _x.cols());
		x(0, 0) = 1;
		x.block(1, 0, _x.rows(), 1) = _x;
		return x;
	}
	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	Eigen::MatrixXd NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::normalizeDataset(const Eigen::MatrixXd &_set) {
		// Calculate medias
		for (unsigned i = 0; i < InputSize_; i++) {
			mNormalizeParameters.first(0,i) = _set.block(0, i, _set.rows(), 1).sum()/_set.rows();
		}

		// Calculate sigmas
		for (unsigned i = 0; i < InputSize_; i++) {
			Eigen::MatrixXd vNu(_set.rows(),1);
			vNu.fill(mNormalizeParameters.first(0,i));
			auto diff = _set.block(0, i, _set.rows(), 1) - vNu;
			mNormalizeParameters.second(0,i) = sqrt(diff.cwiseProduct(diff).sum()/_set.rows());
		}
		auto set = _set;

		for (int i = 0; i < _set.rows(); i++) {
			set.block<1, InputSize_>(i,0) = normalizeInput(_set.block<1, InputSize_>(i,0));
		}
		return set;
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	Eigen::Matrix<double, InputSize_, 1> NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::normalizeInput(const Eigen::Matrix<double, InputSize_, 1> &_x){
		auto x = _x;
		for (unsigned i = 0; i < InputSize_; i++) {
			x(i,0) = (_x(i,0)-mNormalizeParameters.first(0,i))/mNormalizeParameters.second(0,i); // x = (x-nu)/sigma
		}
		return x;
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	Eigen::MatrixXd NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::logarithm(const Eigen::MatrixXd &_in) {
		Eigen::MatrixXd res = _in;
		for (int i = 0; i < _in.rows(); i++) {
			for (int j = 0; j < _in.cols(); j++) {
				res(i,j) = log(_in(i,j));
			}
		}
		return res;
	}
	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	Eigen::MatrixXd NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::sigmoid(const Eigen::MatrixXd &_in) {
		Eigen::MatrixXd res = _in;
		for (int i = 0; i < _in.rows(); i++) {
			for (int j = 0; j < _in.cols(); j++) {
				res(i,j) = 1/(1+exp(-_in(i,j)));
			}
		}
		return res;
	}

	//-------------------------------------------------------------------------------------------------------------
	template<unsigned InputSize_, unsigned HiddenLayers_, unsigned HiddenUnits_, unsigned OutputSize_>
	Eigen::MatrixXd NeuronalNetwork<InputSize_, HiddenLayers_, HiddenUnits_, OutputSize_>::sigmoidGradient(const Eigen::MatrixXd &_in) {
		return sigmoid(_in).cwiseProduct((MatrixXd::Ones(_in.rows(), _in.cols())-sigmoid(_in)));;
	}
}