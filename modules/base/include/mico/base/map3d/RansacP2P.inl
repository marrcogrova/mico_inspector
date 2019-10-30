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

// Algorithm based on "RGB-D Mapping : Using Depth Cameras for
// Dense 3D Modeling of Indoor Environments" by Henry, Peter; 
// Krainin, Michael; Herbst, Evan; Ren, Xiaofeng; Fox, Dieter.

#include <time.h>
#include <random>

#include <opencv2/opencv.hpp>
#include <pcl/registration/transformation_estimation_svd.h>

namespace mico {
	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void RansacP2P<PointType_>::source(const pcl::PointCloud<PointType_>& _source, const cv::Mat &_descriptors) {
        mSource = _source;
        _descriptors.copyTo(mSourceDescriptors);
	}

	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void RansacP2P<PointType_>::target(const pcl::PointCloud<PointType_>& _target, const cv::Mat &_descriptors) {
        mTarget = _target;
        _descriptors.copyTo(mTargetDescriptors);
	}

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void RansacP2P<PointType_>::sourceTarget(const pcl::PointCloud<PointType_> &_source, const pcl::PointCloud<PointType_> &_target, const std::vector<cv::DMatch> &_matches){
        mTarget = _target;
        mSource = _source;
        mMatches = _matches;
    }

	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void RansacP2P<PointType_>::maxIters(unsigned _iters) {
		mMaxIters = _iters;
	}

	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned RansacP2P<PointType_>::maxIters() {
		return mMaxIters;
	}

	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void RansacP2P<PointType_>::maxDistance(double _distance) {
        mMaxDistance = _distance;
	}

	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double RansacP2P<PointType_>::maxDistance() {
        return mMaxDistance;
	}

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void RansacP2P<PointType_>::minInliers(int _min){
        mMinInliers =_min;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    int RansacP2P<PointType_>::minInliers() {
        return mMinInliers;
    }

	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void RansacP2P<PointType_>::inliers(std::vector<int>& _inliers) {
		_inliers = mInliers;
	}

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void RansacP2P<PointType_>::inliers(std::vector<cv::DMatch> &_inliers){
        _inliers = mInliersMatches;
    }

	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double RansacP2P<PointType_>::score() {
		return mMaxIters;
	}

	//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool RansacP2P<PointType_>::run() {
		// Check arguments
        if (mSource.size() == 0 || mTarget.size() == 0) {
            std::cout << "[RANSACP2P] Error, Input clouds empty" << std::endl;
			return false;
		}
        if(mMatches.size() == 0){
            if( mSourceDescriptors.rows == 0 || mTargetDescriptors.rows == 0){
                if(mMatches.size() == 0){
                    std::cout << "[RANSACP2P] Error, neither descriptors or matches are filled" << std::endl;
                    return false;
                }
            }else{
                std::vector<cv::DMatch> matches12, matches21;   // 666 TODO: Do it cleaner and better
                cv::FlannBasedMatcher featureMatcher;
                featureMatcher.match(mSourceDescriptors, mTargetDescriptors, matches12);
                featureMatcher.match(mTargetDescriptors, mSourceDescriptors, matches21);

                double max_dist = 0; double min_dist = 999999;
                //-- Quick calculation of max and min distances between keypoints
                for( int i = 0; i < mSourceDescriptors.rows; i++ ) {
                    double dist = matches12[i].distance;
                    if( dist < min_dist ) min_dist = dist;
                    if( dist > max_dist ) max_dist = dist;
                }

                // symmetry test.
                for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
                    for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
                        if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                            if(it12->distance <= min_dist*8){   // 666 TODO: parametrize
                                mMatches.push_back(*it12);
                            }
                            break;
                        }
                    }
                }
            }
        }

        std::cout << "Total matches: " << mMatches.size() << std::endl;
		// Compute matches between point clouds.
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transformations(mMaxIters);
        std::vector<double> scores(mMaxIters, 0);
        std::vector<std::vector<cv::DMatch>> inlierMatches(mMaxIters);

        // Generate possible combinations to avoid repetitions
        // 666 TODO: improve!
        std::vector<std::vector<int>> combinations(mMaxIters);

        srand(time(NULL));	// 666 TODO if it takes long time, move to another place!
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, mMatches.size()-1);

        ///std::ofstream file("/home/bardo91/Desktop/scores.txt");
        ///std::ofstream fileT("/home/bardo91/Desktop/transformations.txt");

        // Generate all random numbers
        std::vector<int> rndNumbers(mMaxIters); // 666 TODO: not sure if thread safe so generating all in once
        for(int i = 0; i < mMaxIters; i++){
			for (unsigned j = 0; j < 3; j++) {
				combinations[i].push_back(dis(gen));
			}
        }

		// Sample points.
        // #pragma omp parallel for
        for (int i = 0; i < mMaxIters; i++) {
            // int i = omp_get_thread_num();
            // Take 3 random matches.
            pcl::PointCloud<PointType_> src, tgt;
            pcl::Correspondences correspondences;
            for (unsigned pi = 0; pi < 3; pi++) {
                cv::DMatch rndMatch = mMatches[combinations[i][pi]];
                src.push_back(mSource[rndMatch.queryIdx]);
                tgt.push_back(mTarget[rndMatch.trainIdx]);
                correspondences.push_back(pcl::Correspondence(pi,pi, 0));
            }
            // Compute rigid transform. Using Umeyama estimator
            pcl::registration::TransformationEstimationSVD<PointType_, PointType_, float> transformationEstimator;
            transformationEstimator.estimateRigidTransformation(src, tgt, correspondences, transformations[i]);

            // Rotate cloud
            pcl::PointCloud<PointType_> transformedCloud;
            pcl::transformPointCloud(mSource, transformedCloud, transformations[i]);

            // Reject outliers, compute cost and store result. 666 Build KDTree for a fast searching!
            for(auto match: mMatches){
                PointType_ pSrc = transformedCloud[match.queryIdx];
                PointType_ pTgt = mTarget[match.trainIdx];
                float diffX = pTgt.x - pSrc.x, diffY = pTgt.y - pSrc.y, diffZ = pTgt.z - pSrc.z;
                double dist = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);

                if (dist < mMaxDistance) {	// Inlier
                    inlierMatches[i].push_back(match);
                    scores[i] += dist;
                }
                else {	// Outlier
                    // Nothing to do.
                }
            }
            // Normalize score.
            scores[i] /= inlierMatches[i].size();

            ///file << scores[i] << "\t" << inlierMatches[i].size() << std::endl;
            ///
            ///fileT << transformations[i]<<std::endl;
            ///
            ///cv::Mat cpyMatches;
            ///cv::hconcat(srcKf.left, tgtKf.left, cpyMatches);
            ///
            ///for (unsigned pi = 0; pi < 3; pi++) {
            ///    cv::DMatch match = mMatches[combinations[rndId][pi]];
            ///    auto p1 = srcKf.featureProjections[match.queryIdx];
            ///    auto p2 = tgtKf.featureProjections[match.trainIdx] + cv::Point2f(640,0);
            ///    cv::circle(cpyMatches,p1, 3, cv::Scalar(0,255, 0),3);
            ///    cv::circle(cpyMatches,p2, 3, cv::Scalar(0,0,255),3);
            ///    cv::line(cpyMatches, p1, p2, cv::Scalar(255,255,255),1);
            ///}
            ///imshow("matchesaa", cpyMatches);

            ///////////////////////// VISUALIZATION
            ///std::cout << "score: " << scores[i] << "\t" << "inliers: " << inlierMatches[i].size() << std::endl;
            ///std::string nameori = "ori"+std::to_string(i);
            ///std::string name = "result"+std::to_string(i);
            ///pcl::visualization::PCLVisualizer viewerori(nameori.c_str());
            ///pcl::visualization::PCLVisualizer viewer(name.c_str());
            ///Eigen::Affine3f pose;
            ///pose.matrix() = Eigen::Matrix4f::Identity();
            ///viewer.addCoordinateSystem(0.15, pose, "camera1");
            ///viewer.addPointCloud(mTarget.makeShared(),"c1");
            ///viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c1");
            ///viewerori.addCoordinateSystem(0.15, pose, "ocamera1");
            ///viewerori.addPointCloud(mTarget.makeShared(),"oc1");
            ///viewerori.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "oc1");
            ///viewerori.addPointCloud(mSource.makeShared(),"oc2");
            ///viewerori.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "oc2");
            ///
            ///for(unsigned pi = 0; pi < 3; pi++){
            ///    tgt[pi].b=0;tgt[pi].g=0;tgt[pi].r=255;
            ///    src[pi].b=0;src[pi].g=255;src[pi].r=0;
            ///    viewer.addLine(tgt[pi], src[pi], "line_"+std::to_string(pi));
            ///    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH , 5, "line_"+std::to_string(pi));
            ///}
            ///
            ///viewerori.addPointCloud(tgt.makeShared(),"cc1");
            ///viewerori.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cc1");
            ///viewerori.addPointCloud(src.makeShared(),"cc2");
            ///viewerori.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cc2");
            ///
            ///pose.matrix() = transformations[i];
            ///auto copySrc = mSource;
            ///viewer.addCoordinateSystem(0.1, pose, "camera2");
            ///pcl::transformPointCloud(copySrc, copySrc, transformations[i]);
            ///viewer.addPointCloud(copySrc.makeShared(),"c2");
            ///viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c2");
            ///
            ///for(unsigned pi = 0; pi < 3; pi++){
            ///    tgt[pi].b=0;tgt[pi].g=0;tgt[pi].r=255;
            ///    src[pi].b=0;src[pi].g=255;src[pi].r=0;
            ///    //viewer.addLine(tgt[pi], src[pi], "line_"+std::to_string(pi));
            ///    //viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH , 5, "line_"+std::to_string(pi));
            ///}
            ///
            ///
            ///viewer.addPointCloud(tgt.makeShared(),"cc1");
            ///viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cc1");
            ///pcl::transformPointCloud(src, src, transformations[i]);
            ///viewer.addPointCloud(src.makeShared(),"cc2");
            ///viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cc2");
            ///
            ///
            ///std::cout << transformations[i] << std::endl;
            ///
            ///while(!viewer.wasStopped()){
            ///   viewer.spinOnce(30);
            ///   std::this_thread::sleep_for(std::chrono::milliseconds(30));
            ///   cv::waitKey(3);
            ///}
            ////////////// VISUALIZATION

            for(auto match: mMatches){
                PointType_ pSrc = transformedCloud[match.queryIdx];
                PointType_ pTgt = mTarget[match.trainIdx];
                float diffX = pTgt.x - pSrc.x, diffY = pTgt.y - pSrc.y, diffZ = pTgt.z - pSrc.z;
                double dist = sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);

                if (dist < mMaxDistance) {	// Inlier

                }
                else {	// Outlier
                    // Nothing to do.
                }
            }

        } // End omp parallel for
        

		// Choose best option.
        const int minInliers = mMatches.size() * 0.10 < mMinInliers? mMinInliers: mMatches.size() * 0.10;   // 666 TODO_ what to do with the  % of inliers
		double minScore = 9999999;
        int minIndex = -1;
		for (unsigned i = 0; i < scores.size(); i++) {
            if (scores[i] < minScore && inlierMatches[i].size() > minInliers) {
                minScore = scores[i];
                minIndex = i;
			}
		}
        std::cout << "MinIdx: " << minIndex << ", score: " << minScore<< ", inliers: " << inlierMatches[minIndex].size() << std::endl;
        if(minIndex == -1){
            return false;
        }else{
            //mInliers = inliers[minIndex];
            mLastScore = scores[minIndex];
            mLastTransformation = transformations[minIndex];
            mInliersMatches = inlierMatches[minIndex];

            return true;
        }
	}

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline Eigen::Matrix4f RansacP2P<PointType_>::transformation(){
        return mLastTransformation;
    }

}	//	namespace mico 
