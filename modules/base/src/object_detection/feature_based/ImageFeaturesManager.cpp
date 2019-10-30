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


#include <mico/base/object_detection/feature_based/ImageFeaturesManager.h>
// #include <opencv2/xfeatures2d.hpp>
namespace mico {
	//----------------------------------------------------------------------------------------------------------------------
	bool ImageFeatureManager::configure(const cjson::Json & _configuration) {
		if(_configuration["detector"] == "FAST"){
			mFeatureDetector = cv::FastFeatureDetector::create((int)_configuration["params"]["fastThreshold"]);
			mDetector = eDetector::FAST;
		}else if(_configuration["detector"] == "ORB"){
			mFeatureDetector = cv::ORB::create();
			mDetector = eDetector::ORB;
		}else if(_configuration["detector"] == "ORB_SLAM"){
			mpORBextractor = new ORB_SLAM2::ORBextractor(500,1.2,4,30,7);
			mDetector = eDetector::ORB_SLAM;
		}else if(_configuration["detector"] == "SIFT"){
			assert(false);
			// mFeatureDetector = cv::xfeatures2d::SIFT::create();
			// mDetector = eDetector::SIFT;
		}else if(_configuration["detector"] == "SURF"){
			assert(false);
			// mFeatureDetector = cv::xfeatures2d::SURF::create();
			// mDetector = eDetector::SURF;
		}else{
			std::cout << "[IMAGE FEATURE MANAGER] Error during configuration: Unknown type of detector." << std::endl;
			return false;
		}

		if(_configuration["descriptor"] == "BRIEF"){
			assert(false);
			// mFeatureDescriptor = cv::xfeatures2d::BriefDescriptorExtractor::create(32, false);
			// mMatcher = new cv::BFMatcher(cv::NORM_HAMMING);
			// mDescriptor = eDescriptor::BRIEF;
		}else if(_configuration["descriptor"] == "rBRIEF"){
			assert(false);
			// mFeatureDescriptor = cv::xfeatures2d::BriefDescriptorExtractor::create(32, true);
			// mMatcher = new cv::BFMatcher(cv::NORM_HAMMING);
			// mDescriptor = eDescriptor::rBRIEF;
		}else if(_configuration["descriptor"] == "ORB"){
			mFeatureDescriptor = cv::ORB::create();
			mMatcher = new cv::BFMatcher(cv::NORM_HAMMING);
			mDescriptor = eDescriptor::ORB;
		}else if(_configuration["detector"] == "ORB_SLAM"){
			mMatcher = new cv::BFMatcher(cv::NORM_HAMMING);
			mDescriptor = eDescriptor::ORB_SLAM;
		}else if(_configuration["descriptor"] == "SIFT"){
			assert(false);
			// mFeatureDescriptor = cv::xfeatures2d::SIFT::create();
			// mMatcher = new cv::FlannBasedMatcher();
			// mDescriptor = eDescriptor::SIFT;
		}else if(_configuration["descriptor"] == "SURF"){
			assert(false);
			// mFeatureDescriptor = cv::xfeatures2d::SURF::create();
			// mMatcher = new cv::FlannBasedMatcher();
			// mDescriptor = eDescriptor::SURF;
		}else {
			std::cout << "[IMAGE FEATURE MANAGER] Error during configuration: Unknown type of descriptor." << std::endl;
			return false;
		}

		if(_configuration.contains("maxDistance")){
			mMaxMatchingDistance = (int) _configuration["maxDistance"];
		}

		return true;
	}

	//----------------------------------------------------------------------------------------------------------------------
	bool ImageFeatureManager::compute(const cv::Mat & _frame, std::vector<cv::Point2f>& _points, cv::Mat & _descriptors, cv::Rect _roi) {
		std::vector<cv::KeyPoint> kps;			
		cv::Mat image = _frame(_roi).clone();
		cv::cvtColor(image, image, CV_BGR2GRAY);
		if(mDetector!= eDetector::ORB_SLAM){	
			mFeatureDetector->detect(image, kps);
			mFeatureDescriptor->compute(image, kps, _descriptors);
			//_descriptors.convertTo(_descriptors, CV_32F);
		}else{
			(*mpORBextractor)(image,cv::Mat(),kps,_descriptors);
		}
		for(auto &kp:kps){
			_points.push_back(kp.pt + cv::Point2f(_roi.x, _roi.y));
		}
		if(kps.size() == 0){
			return false;
		}else{
			return true;
		}
	}

	//----------------------------------------------------------------------------------------------------------------------
	bool ImageFeatureManager::match(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_matches) {
		if(mDescriptor == eDescriptor::ORB_SLAM){
			auto distFeatures = [](const cv::Mat &a, const  cv::Mat &b,int i, int j){
				const int *pa = a.ptr<int32_t>(i);
				const int *pb = b.ptr<int32_t>(j);
				int dist=0;

				for(int i=0; i<8; i++, pa++, pb++) {
					unsigned  int v = *pa ^ *pb;
					v = v - ((v >> 1) & 0x55555555);
					v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
					dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
				}

				return dist;
			};
			
			std::vector<cv::DMatch> matches12(_des1.rows), matches21(_des2.rows);

			int counter1 = 0;
			for(int i = 0; i < _des1.rows; i++){
				cv::DMatch minIdx1, minIdx2; 
				int minDist1 = 99999, minDist2 = 9999;
				for(int j = 0; j < _des2.rows; j++){
					int dist = distFeatures(_des1, _des2,i,j);
					if(dist < minDist1){
						minDist1 = dist;
						minIdx1.queryIdx = i;
						minIdx1.trainIdx = j;
						minIdx1.distance = dist;
					}else{
						if(dist < minDist2){
							minDist2 = dist;
							minIdx2.queryIdx = i;
							minIdx2.trainIdx = j;
							minIdx2.distance = dist;
						}
					}
				}
				if(minIdx1.distance < minIdx2.distance*0.9){
					matches12[counter1] = minIdx1;
					counter1++;
				}
			}
			matches12.resize(counter1);

			int counter2 = 0;
			for(int i = 0; i < _des2.rows; i++){
				cv::DMatch minIdx1, minIdx2; 
				int minDist1 = 99999, minDist2 = 9999;
				for(int j = 0; j < _des1.rows; j++){
					int dist = distFeatures(_des2, _des1,i,j);
					if(dist < minDist1){
						minDist1 = dist;
						minIdx1.queryIdx = i;
						minIdx1.trainIdx = j;
						minIdx1.distance = dist;
					}else{
						if(dist < minDist2){
							minDist2 = dist;
							minIdx2.queryIdx = i;
							minIdx2.trainIdx = j;
							minIdx2.distance = dist;
						}
					}
				}
				if(minIdx1.distance < minIdx2.distance*0.9){
					matches21[counter2] = minIdx1;
					counter2++;
				}
			}		
			matches21.resize(counter2);
		

			// symmetry test.
			for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
				for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
					if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
						_matches.push_back(*it12);
						break;
					}
				}
			}
		}else{
			std::vector<std::vector<cv::DMatch>> matches12, matches21;
			mMatcher->knnMatch(_des1, _des2, matches12,2);
			mMatcher->knnMatch(_des2, _des1, matches21,2);

			std::vector<cv::DMatch> matches12fil, matches21fil; 
			for(auto &matches: matches12){
                if(matches[0].distance < matches[1].distance * 0.9){
                    matches12fil.push_back(matches[0]);
                }
            }
 
            for(auto &matches: matches21){
                if(matches[0].distance < matches[1].distance * 0.9){
                    matches21fil.push_back(matches[0]);
                }
            }

			for(std::vector<cv::DMatch>::iterator it12 = matches12fil.begin(); it12 != matches12fil.end(); it12++){
            for(std::vector<cv::DMatch>::iterator it21 = matches21fil.begin(); it21 != matches21fil.end(); it21++){
                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                    _matches.push_back(*it12);
                    break;
                }
            }
        }
		}
		return true;
	}
}
