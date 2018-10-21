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

namespace rgbd{
	
	//---------------------------------------------------------------------------------------------------------------------
	template<typename ParticleType_>
	void ParticleFilterCPU<ParticleType_>::step(ParticleType_ &_realParticle) {
		simulate();
		calcWeigh(_realParticle);
		resample();
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename ParticleType_>
	void ParticleFilterCPU<ParticleType_>::init(){
		for (unsigned i = 0; i < mNuParticles; i++){
			mParticles.push_back(ParticleType_());
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename ParticleType_>
	void ParticleFilterCPU<ParticleType_>::simulate() {
		for (unsigned i = 0; i < mNuParticles; i ++) {
			mParticles[i].simulate();
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename ParticleType_>
	void ParticleFilterCPU<ParticleType_>::calcWeigh(ParticleType_ &_realParticle) {
		for (unsigned i = 0; i < mNuParticles; i++) {
			mParticles[i].calcWeigh(_realParticle);
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename ParticleType_>
	void ParticleFilterCPU<ParticleType_>::resample() {
		std::vector<ParticleType_> newParticles;
		double beta = 0.0;
		unsigned index = unsigned(double(rand()) / RAND_MAX * mNuParticles);
		double maxWeigh = 0.0;

		for (unsigned i = 0; i < mNuParticles; i++) {
			if (mParticles[i].weigh() > maxWeigh)
				maxWeigh = mParticles[i].weigh();
		}

		for (unsigned i = 0; i < mNuParticles; i++) {
			beta += double(rand()) / RAND_MAX * 2.0 * maxWeigh;
			while (beta > mParticles[index].weigh()) {
				beta -= mParticles[index].weigh();
				index = (index + 1) % mNuParticles;
			}
			newParticles.push_back(mParticles[index]);
		}
		
		mParticles = newParticles;
	}
}