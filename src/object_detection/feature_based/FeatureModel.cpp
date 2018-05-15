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

#include <rgbd_tools/object_detection/feature_based/FeatureModel.h>
#include <opencv2/opencv.hpp>
#include <chrono>
using namespace cv;

namespace rgbd
{

//---------------------------------------------------------------------------------------------------------------------
bool FeatureModel::init(const cjson::Json &_configuration)
{

	mRansacIterations = (int)_configuration["ransac"]["iterations"];
	mInliersThreshold = (int)_configuration["ransac"]["minInliers"];
	mRansacErr = (float)_configuration["ransac"]["maxRepError"];
	mRansacConf = (float)_configuration["ransac"]["confidence"];

	std::cout << "Configuring feature detector and descriptor --- ";
	if (!mImageFeatureManager.configure(_configuration["features"]))
	{
		std::cout << "Error configuring image features manager" << std::endl;
		return false;
	}
	std::cout << " >>> configured" << std::endl;
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool FeatureModel::save(std::string _modelName)
{
	FileStorage fs(_modelName, FileStorage::WRITE);
	if (!fs.isOpened())
		return false;

	fs << "points" << mPoints;
	fs << "descriptors" << mDescriptors;
	fs.release();
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool FeatureModel::load(std::string _modelName)
{
	FileStorage fs(_modelName, FileStorage::READ);
	if (!fs.isOpened())
		return false;

	fs["points"] >> mPoints;
	fs["descriptors"] >> mDescriptors;
	fs.release();
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool FeatureModel::find(cv::Mat &_image, const cv::Mat &_intrinsic, const cv::Mat &_coeff, cv::Mat &_position, cv::Mat &_orientation, std::vector<cv::Point2f> &_inliers, bool _useGuess, cv::Rect _roi)
{
	// Look for features in the window.
	std::vector<cv::Point2f> scenePoints;
	cv::Mat sceneDescriptors;

	auto t0 = std::chrono::high_resolution_clock::now();
	mImageFeatureManager.compute(_image, scenePoints, sceneDescriptors, _roi);
	std::cout << "found: " << scenePoints.size() << " features" << std::endl;
	if (scenePoints.size() < mInliersThreshold)
	{ // 666 TODO: parametrize this value
		return false;
	}
	auto t1 = std::chrono::high_resolution_clock::now();

	// Match model features with scene features.
	std::vector<cv::DMatch> matches;
	mImageFeatureManager.match(mDescriptors, sceneDescriptors, matches);
	std::cout << "found: " << matches.size() << " matches" << std::endl;
	if (matches.size() < mInliersThreshold)
	{ // 666 TODO: parametrize  this value
		return false;
	}
	auto t2 = std::chrono::high_resolution_clock::now();

	std::vector<cv::Point3f> modelFeaturesMatched;
	std::vector<cv::Point2f> sceneFeaturesMatched;
	for (auto &match : matches)
	{
		modelFeaturesMatched.push_back(mPoints[match.queryIdx]);
		sceneFeaturesMatched.push_back(scenePoints[match.trainIdx]);
	}
	auto t3 = std::chrono::high_resolution_clock::now();

	// Perform PnP solver
	std::vector<int> inliersIdx;
	cv::Mat position, orientation;
	cv::solvePnPRansac(modelFeaturesMatched, sceneFeaturesMatched,			  // Input features matched
					   _intrinsic, _coeff,									  // Camera calibration
					   _orientation, _position,								  // Output object pose
					   _useGuess, mRansacIterations, mRansacErr, mRansacConf, // Algorithm configuration
					   inliersIdx, cv::SOLVEPNP_EPNP);						  // inlier/outlier list (1 or 0)

	std::cout << "found: " << inliersIdx.size() << " inliers" << std::endl;
	auto t4 = std::chrono::high_resolution_clock::now();
	//std::cout << "RANSAC: features: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count() << std::endl;
	//std::cout << "RANSAC: matches: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << std::endl;
	//std::cout << "RANSAC: ransac: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4-t3).count() << std::endl;
	if (inliersIdx.size() > mInliersThreshold)
	{
		for (auto &inlierIdx : inliersIdx)
		{
			cv::circle(_image, sceneFeaturesMatched[inlierIdx], 3, cv::Scalar(0, 0, 255), 3);
			_inliers.push_back(sceneFeaturesMatched[inlierIdx]);
		}

		return true;
	}
	else
	{
		return false;
	}
}
} // namespace rgbd
