//
//
//
//
//
#ifdef USE_CVSBA
	#include <cvsba/cvsba.h>
#endif

#include <thread>
#include <chrono>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>


namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::addKeyframe(const Keyframe<PointType_> &_kf){

        auto kf = _kf;
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        if((_kf.featureCloud == nullptr || _kf.featureCloud->size() ==0)) {
            if(mKeyframes.size() != 0){
                auto t1 = std::chrono::high_resolution_clock::now();
                // Fine rotation.
                if(!refineTransformation( mKeyframes.back(), kf, transformation)){
                    return false;   // reject keyframe.
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<"Refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;
            }
        }else{
            if(mKeyframes.size() == 0){
                initDataBA(kf);
            }else{
                // Match feature points.
                auto t0 = std::chrono::high_resolution_clock::now();
                // Compute initial rotation.
                if(!transformationBetweenFeatures( mKeyframes.back(), kf, transformation)){
                    return false;   // reject keyframe.
                }

                auto t1 = std::chrono::high_resolution_clock::now();

                if(mIcpEnabled){
                    // Fine rotation.
                    if(!refineTransformation( mKeyframes.back(), kf, transformation)){
                        return false;   // reject keyframe.
                    }
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<	"\trough: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() <<
                                ", refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;

                // Extend covisibility matrix and feature points for BA.
                //extendDataBA(kf);
            }
        }
        Eigen::Affine3f currentPose;
        if(mKeyframes.size() != 0){
            Eigen::Affine3f prevPose = Eigen::Translation3f(mKeyframes.back().position)*mKeyframes.back().orientation;
            Eigen::Affine3f lastTransformation(transformation);
            // Compute current position.
            currentPose = lastTransformation*prevPose;

            kf.position = currentPose.translation();
            kf.orientation = currentPose.rotation();

            // Check transformation
            Eigen::Vector3f ea = transformation.block<3,3>(0,0).eulerAngles(0, 1, 2);
            if(ea[0] > 20 || ea[1] > 20 || ea[2] > 20 || transformation.block<3,1>(0,3).norm() > 0.3){
                std::cout << "Large transformation! not accepted KF" << std::endl;
                return false;
            }

            pcl::PointCloud<PointType_> cloud;
            pcl::transformPointCloudWithNormals(*kf.cloud, cloud, currentPose);
            mMap += cloud;
        }

        // Add keyframe to list.
        mKeyframes.push_back(kf);

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>>>  SceneRegistrator<PointType_>::keyframes() const{
        return mKeyframes;
    }


    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    pcl::PointCloud<PointType_> SceneRegistrator<PointType_>::map() const{
        return mMap;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::optimize(){
		#ifdef USE_CVSBA
			std::vector<cv::Mat> ts, rs, intrinsics, coeffs;
			for(auto kf: mKeyframes){
				intrinsics.push_back(kf.intrinsic);
				coeffs.push_back(kf.coefficients);
				ts.push_back(cv::Mat(3,1,CV_32F, &kf.position[0]));
				auto rotation = kf.orientation.matrix();
				auto cvRotation = cv::Mat(3,3,CV_32F, rotation.data());
				cv::Rodrigues(cvRotation, cvRotation);
				rs.push_back(cvRotation);
			}

			// Initialize cvSBA and perform bundle adjustment.
			cvsba::Sba bundleAdjuster;
			cvsba::Sba::Params params;
			params.verbose = true;
			params.iterations = mBaIterations;
			params.minError = mBaMinError;
			bundleAdjuster.setParams(params);

			assert( mSceneFeaturePoints.size() == mSceneFeatureProjections[0].size() &&
					mCovisibilityMatrix.cols == mSceneFeaturePoints.size() &&
					mCovisibilityMatrix.rows == mSceneFeatureProjections.size() &&
					intrinsics.size() == mSceneFeatureProjections.size() &&
					intrinsics.size() == coeffs.size() &&
					intrinsics.size() == rs.size() &&
					ts.size() == rs.size());

			// count aparitions
			std::vector<int> aparitions(mCovisibilityMatrix.cols);
			for(unsigned i = 0; i < mCovisibilityMatrix.rows; i++){
				for(unsigned j = 0; j < mCovisibilityMatrix.cols; j++){
					aparitions[j] += mCovisibilityMatrix.at<int>(i,j);
				}
			}


			// Create data only for elementes with enough aparitions. 666 TODO improve mem alloc! possible bottle neck
			std::vector<std::vector<int>> visibility(mCovisibilityMatrix.rows);
			std::vector<cv::Point3f> points;
			std::vector<std::vector<cv::Point2f>> projections(mCovisibilityMatrix.rows);
			for(unsigned i = 0; i < aparitions.size(); i++){
                if(aparitions[i] > mBaMinAparitions){
					points.push_back(mSceneFeaturePoints[i]);
					for(unsigned j = 0; j < mCovisibilityMatrix.rows; j++){
						projections[j].push_back(mSceneFeatureProjections[j][i]);
						visibility[j].push_back(mCovisibilityMatrix.at<int>(j,i));
					}
				}
			}

			/////std::cout << "----------PROJECTIONS--------------" << std::endl;
			///
			/////////////////
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
			viewer1->addCoordinateSystem (1.0);

			for(unsigned i = 0; i < mKeyframes.size(); i++){
				auto kf = mKeyframes[i];

				pcl::visualization::PointCloudColorHandlerCustom<PointType_> single_color(kf.featureCloud, 255/mKeyframes.size()*i, 255-255/mKeyframes.size()*i, 0);
				viewer1->addPointCloud<PointType_> (kf.featureCloud, single_color, "cloud_"+std::to_string(i));
				viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_"+std::to_string(i));

				Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
				transformation.block<3,3>(0,0) = kf.orientation.matrix();
				transformation.block<3,1>(0,3) = kf.position;

				Eigen::Affine3f pose;
				pose.matrix() = transformation;
				viewer1->addCoordinateSystem(0.5, pose, "camera_" + std::to_string(i));

				if(i < mKeyframes.size()-1){
					cv::Mat matchResult = mKeyframes[i].left;
					cv::Mat R,T;    // 666 Not used
					cv::hconcat(matchResult, mKeyframes[i+1].left, matchResult);
					std::vector<cv::DMatch> matches;
					matchDescriptors(mKeyframes[i].featureProjections, mKeyframes[i+1].featureProjections, mKeyframes[i].featureDescriptors, mKeyframes[i+1].featureDescriptors, matches, R, T);
					for(auto match: matches){
						viewer1->addLine(   mKeyframes[i].featureCloud->at(match.queryIdx),
											mKeyframes[i+1].featureCloud->at(match.trainIdx),
											"line"+std::to_string(i)+std::to_string(match.queryIdx));

						cv::line(   matchResult,
									mKeyframes[i].featureProjectionsColor[match.queryIdx],
									cv::Point2f(640,0) + mKeyframes[i+1].featureProjectionsColor[match.trainIdx],
									cv::Scalar(0,255,0), 1);
					}
					cv::namedWindow("resmatches"+std::to_string(i), CV_WINDOW_FREERATIO);
					cv::imshow("resmatches"+std::to_string(i), matchResult);
				}

				viewer1->addPointCloud(mKeyframes[i].cloud,"clouddense_"+std::to_string(i));
				viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_"+std::to_string(i));
			}

			while(!viewer1->wasStopped()){
			   viewer1->spinOnce(30);
			   std::this_thread::sleep_for(std::chrono::milliseconds(30));
			   cv::waitKey(3);
			}

			///////////////////
			bundleAdjuster.run(points, projections, visibility, intrinsics, rs, ts, coeffs);


			//
			pcl::visualization::PCLVisualizer viewer("result");
			pcl::PointCloud<PointType_> cloud;

			for(unsigned i = 0; i < points.size(); i++){
			   PointType_ point;
			   point.x = mSceneFeaturePoints[i].x;
			   point.y = mSceneFeaturePoints[i].y;
			   point.z = mSceneFeaturePoints[i].z;
			   point.r = double(rand())/RAND_MAX*255;
			   point.g = double(rand())/RAND_MAX*255;
			   point.b = double(rand())/RAND_MAX*255;

			   cloud.push_back(point);
			}

			viewer.addPointCloud(cloud.makeShared(),"cloud");
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"cloud");

			for(unsigned i = 0; i < ts.size(); i++){
			   Eigen::Affine3f pose;
			   cv::Mat R;
			   cv::Rodrigues(rs[i], R);

			   Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

			   rotation(0,0) = R.at<double>(0,0);
			   rotation(0,1) = R.at<double>(0,1);
			   rotation(0,2) = R.at<double>(0,2);
			   rotation(1,0) = R.at<double>(1,0);
			   rotation(1,1) = R.at<double>(1,1);
			   rotation(1,2) = R.at<double>(1,2);
			   rotation(2,0) = R.at<double>(2,0);
			   rotation(2,1) = R.at<double>(2,1);
			   rotation(2,2) = R.at<double>(2,2);
			   rotation(0,3) = ts[i].at<double>(0);
			   rotation(1,3) = ts[i].at<double>(1);
			   rotation(2,3) = ts[i].at<double>(2);

			   pose.matrix() = rotation;

			   viewer.addCoordinateSystem(0.15, pose, "camera_" + std::to_string(i));


			   Eigen::Matrix3f r = pose.matrix().block<3,3>(0,0);
			   pose.matrix().block<3,3>(0,0) = r.inverse();
			   pose.matrix().block<3,1>(0,3) = -pose.matrix().block<3,1>(0,3);
			   auto cloudKf = *mKeyframes[i].cloud;
			   pcl::transformPointCloud(cloudKf, cloudKf, pose.matrix());
			   viewer.addPointCloud(cloudKf.makeShared(),"cloud_"+std::to_string(i));
			   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_"+std::to_string(i));
			}

			viewer.addCoordinateSystem(0.30, "origin");
			while(!viewer.wasStopped()){
			   viewer.spinOnce(30);
			   std::this_thread::sleep_for(std::chrono::milliseconds(30));
			   cv::waitKey(3);
			}


			//
			return true;
		#else
			std::cout << "CVSBA not installed! CANT PERFORM BA" << std::endl;
			return false;
		#endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::baMinError       () const{
        return mBaMinError;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned SceneRegistrator<PointType_>::baIterations     () const{
        return mBaIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned SceneRegistrator<PointType_>::baMinAparitions  () const{
        return mBaMinAparitions;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::descriptorDistanceFactor     () const{
        return mFactorDescriptorDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline int SceneRegistrator<PointType_>::ransacIterations             () const{
        return mRansacIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::ransacMaxDistance   () const{
        return mRansacMaxDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline int SceneRegistrator<PointType_>::ransacMinInliers   () const{
        return mRansacMinInliers;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::icpMaxTransformationEpsilon() const{
        return mIcpMaxTransformationEpsilon;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::icpMaxCorrespondenceDistance() const{
        return mIcpMaxCorrespondenceDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::icpVoxelDistance() const{
        return mIcpVoxelDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::icpMaxFitnessScore() const{
        return  mIcpMaxFitnessScore;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline int SceneRegistrator<PointType_>::icpMaxIterations() const{
        return mIcpMaxIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baMinError         (double _error){
        mBaMinError = _error;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baIterations       (unsigned _iterations){
        mBaIterations = _iterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baMinAparitions    (unsigned _aparitions){
        mBaMinAparitions = _aparitions;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::descriptorDistanceFactor       (double _factor){
        mFactorDescriptorDistance = _factor;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::ransacIterations               (int _iterations){
        mRansacIterations = _iterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::ransacMaxDistance     (double _maxDistance){
        mRansacMaxDistance = _maxDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::ransacMinInliers     (int _minInliers){
        mRansacMinInliers = _minInliers;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpMaxTransformationEpsilon        (double _maxEpsilon){
        mIcpMaxTransformationEpsilon = _maxEpsilon;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpMaxCorrespondenceDistance       (double _distance){
        mIcpMaxCorrespondenceDistance = _distance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpVoxelDistance     (double _distance){
        mIcpVoxelDistance = _distance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpMaxFitnessScore (double _maxScore){
        mIcpMaxFitnessScore = _maxScore;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpMaxIterations (int _maxIters){
        mIcpMaxIterations = _maxIters;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::matchDescriptors(const std::vector<cv::Point2f> &_kps1, const std::vector<cv::Point2f> &_kps2, const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers, cv::Mat &_R, cv::Mat &_T) {
        std::vector<cv::DMatch> matches12, matches21;
        cv::FlannBasedMatcher featureMatcher;
        featureMatcher.match(_des1, _des2, matches12);
        featureMatcher.match(_des2, _des1, matches21);

        double max_dist = 0; double min_dist = 999999;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < _des1.rows; i++ ) {
            double dist = matches12[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        // symmetry test.
        std::vector<cv::Point2i> pLeft, pRight;
        std::vector<cv::DMatch> symmetricMatches;
        for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
            for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                    if(it12->distance <= min_dist*mFactorDescriptorDistance){
                        pLeft.push_back(_kps1[it12->queryIdx]);
                        pRight.push_back(_kps2[it12->trainIdx]);
                        symmetricMatches.push_back(*it12);
                    }
                    break;
                }
            }
        }

        // Remove outliers using ransac ------------------------------------------------------------
        //cv::Mat mask;
        //cv::Mat E = cv::findEssentialMat(   pLeft, pRight,
        //                                    mKeyframes[0].intrinsic,    // 666 not elegant!
        //                                    cv::RANSAC,
        //                                    mRansacConfidence,
        //                                    mRansacMaxReprojError,
        //                                    mask);
        //
        //for(unsigned i = 0; i < mask.rows; i++){
        //    if(mask.at<uchar>(i) == 1)
        //        _inliers.push_back(symmetricMatches[i]);
        //}
        //
        //cv::Mat intrinsic = mKeyframes[0].intrinsic;
        //cv::recoverPose(E, pLeft, pRight, _R, _T,
        //                intrinsic.at<float>(0,0),
        //                cv::Point2f(intrinsic.at<float>(0,2), intrinsic.at<float>(1,2)),
        //                mask);

    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::initDataBA(const Keyframe<PointType_> &_firstKf){
        mCovisibilityMatrix = cv::Mat(1,_firstKf.featureCloud->size(), CV_32S);
        mSceneFeaturePoints.resize(_firstKf.featureCloud->size());

        std::vector<int> descriptorToCovisibilityIndices(_firstKf.featureCloud->size());
        for(unsigned i = 0; i < _firstKf.featureCloud->size(); i++){
            auto  p = _firstKf.featureCloud->at(i);
            mSceneFeaturePoints[i] = cv::Point3f(p.x, p.y, p.z);
            mCovisibilityMatrix.at<int>(0,i) = 1;
            descriptorToCovisibilityIndices[i] = i;

        }
        mDescriptorToCovisibilityIndices.push_back(descriptorToCovisibilityIndices);
        mSceneFeatureProjections.push_back(_firstKf.featureProjections);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::extendDataBA(const Keyframe<PointType_> &_currentKf){
        // Get matches
        std::vector<cv::DMatch> matches;
        cv::Mat R,T;
        matchDescriptors(_currentKf.featureProjections, mKeyframes.back().featureProjections, _currentKf.featureDescriptors, mKeyframes.back().featureDescriptors, matches, R, T);

        // Get indices of previous keyframe to the covisibility matrix.
        auto descriptorToCovisibilityPrev = mDescriptorToCovisibilityIndices.back();
        std::vector<int> descriptorToCovisibilityCurr(_currentKf.featureCloud->size());

        // Extend covimSceneFeatureProjectionssibility matrix with matched points and add respective projections
        cv::Mat zeroRow = cv::Mat::zeros(1, mCovisibilityMatrix.cols, CV_32S);  // 666 TODO improve!
        vconcat(mCovisibilityMatrix, zeroRow,mCovisibilityMatrix);
        std::vector<cv::Point2f> projections(mCovisibilityMatrix.cols);
        for(auto match: matches){
            mCovisibilityMatrix.at<int>(mCovisibilityMatrix.rows-1, descriptorToCovisibilityPrev[match.trainIdx]) = 1;
            descriptorToCovisibilityCurr[match.queryIdx] = descriptorToCovisibilityPrev[match.trainIdx];
            projections[descriptorToCovisibilityPrev[match.trainIdx]] = _currentKf.featureProjections[match.queryIdx];
        }
        mSceneFeatureProjections.push_back(projections);

        // Extend new points and covisibility matrix and new projections
        int previousCols = mCovisibilityMatrix.cols;
        int noNewPoints = _currentKf.featureCloud->size() - matches.size();
        if(noNewPoints > 0){
            cv::Mat extensionCovisibility = cv::Mat::zeros(mCovisibilityMatrix.rows, noNewPoints, CV_32S);
            hconcat(mCovisibilityMatrix, extensionCovisibility, mCovisibilityMatrix);

            for(auto &vProj: mSceneFeatureProjections){
                vProj.resize(previousCols+noNewPoints, cv::Point2f(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()));
            }
        }
        auto hasMatch = [](int idx, std::vector<cv::DMatch> &_matches){
            bool hasMatch = false;
            for(auto match: _matches){
                if(idx == match.queryIdx) hasMatch = true;
            }
            return hasMatch;
        };

        int newPointIndexCounter = 0;
        for(unsigned i = 0; i < _currentKf.featureCloud->size(); i++){
            if(!hasMatch(i, matches)){
                mCovisibilityMatrix.at<int>(mCovisibilityMatrix.rows-1, previousCols+newPointIndexCounter) = 1;
                descriptorToCovisibilityCurr[i] = previousCols+newPointIndexCounter;
                mSceneFeatureProjections.back()[previousCols+newPointIndexCounter] = _currentKf.featureProjections[i];
                auto p = _currentKf.featureCloud->at(i);
                mSceneFeaturePoints.push_back(cv::Point3f(p.x, p.y, p.z));
                newPointIndexCounter++;
            }
        }
        mDescriptorToCovisibilityIndices.push_back(descriptorToCovisibilityCurr);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool  SceneRegistrator<PointType_>::transformationBetweenFeatures(const Keyframe<PointType_> &_previousKf, const Keyframe<PointType_> &_currentKf, Eigen::Matrix4f &_transformation){
        // Get matches
        std::vector<cv::DMatch> matches12, matches21, matches;   // 666 TODO: Do it once!
        cv::FlannBasedMatcher featureMatcher;
        featureMatcher.match(_currentKf.featureDescriptors, _previousKf.featureDescriptors, matches12);
        featureMatcher.match(_previousKf.featureDescriptors, _currentKf.featureDescriptors, matches21);

        double max_dist = 0; double min_dist = 999999;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < _currentKf.featureDescriptors.rows; i++ ) {
            double dist = matches12[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        // symmetry test.
        for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
            for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                    if(it12->distance <= min_dist*mFactorDescriptorDistance){
                        matches.push_back(*it12);
                    }
                    break;
                }
            }
        }


        ///auto currentTranslated = *_currentKf.cloud;
        ///auto currentFeatureTranslated = *_currentKf.featureCloud;
        ///auto previousFeature = *_previousKf.featureCloud;
        ///
        ///for(unsigned i =0; i < currentTranslated.size(); i++){
        ///    currentTranslated[i].x +=0.25;
        ///}
        ///for(unsigned i =0; i < currentFeatureTranslated.size(); i++){
        ///    currentFeatureTranslated[i].x +=0.25;
        ///    currentFeatureTranslated[i].r = 255;
        ///    currentFeatureTranslated[i].g = 0;
        ///    currentFeatureTranslated[i].b = 0;
        ///}
        ///for(unsigned i =0; i < previousFeature.size(); i++){
        ///    previousFeature[i].r = 0;
        ///    previousFeature[i].g = 255;
        ///    previousFeature[i].b = 0;
        ///}
        ///
        ///pcl::visualization::PCLVisualizer viewerMatches("result");
        ///viewerMatches.addPointCloud(_previousKf.cloud,"c1");
        ///viewerMatches.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c1");
        ///viewerMatches.addPointCloud(currentFeatureTranslated.makeShared(),"ccc1");
        ///viewerMatches.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "ccc1");
        ///Eigen::Affine3f pose1;
        ///pose1.matrix() = Eigen::Matrix4f::Identity();
        ///viewerMatches.addCoordinateSystem(0.15, pose1, "camera1");
        ///viewerMatches.addPointCloud(currentTranslated.makeShared(),"c2");
        ///viewerMatches.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c2");
        ///viewerMatches.addPointCloud(previousFeature.makeShared(),"ccc2");
        ///viewerMatches.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "ccc2");
        ///
        ///cv::Mat cpyMatches;
        ///cv::hconcat(_previousKf.left, _currentKf.left, cpyMatches);
        ///int counter = 0;
        ///for(auto match: matches){
        ///    auto p1 = _previousKf.featureProjections[match.trainIdx];
        ///    auto p2 = _currentKf.featureProjections[match.queryIdx] + cv::Point2f(640,0);
        ///    cv::circle(cpyMatches,p1, 3, cv::Scalar(0,255, 0),3);
        ///    cv::circle(cpyMatches,p2, 3, cv::Scalar(0,0,255),3);
        ///    cv::line(cpyMatches, p1, p2, cv::Scalar(255,255,255),1);
        ///    viewerMatches.addLine(previousFeature[match.trainIdx], currentFeatureTranslated[match.queryIdx], "line_"+std::to_string(counter));
        ///    counter++;
        ///}
        ///imshow("matches", cpyMatches);
        ///
        ///while(!viewerMatches.wasStopped()){
        ///   viewerMatches.spinOnce(30);
        ///   std::this_thread::sleep_for(std::chrono::milliseconds(30));
        ///   cv::waitKey(3);
        ///}

        mRansacAligner.srcKf = _currentKf;
        mRansacAligner.tgtKf = _previousKf;
        mRansacAligner.sourceTarget(*_currentKf.featureCloud, *_previousKf.featureCloud, matches);
        mRansacAligner.maxIters(mRansacIterations);
        mRansacAligner.maxDistance(mRansacMaxDistance);
        mRansacAligner.minInliers(mRansacMinInliers);
        if(!mRansacAligner.run()){
            std::cout << "Cant align clouds using ransac P2P" << std::endl;
            return false;
        }

        _transformation = mRansacAligner.transformation();


        /// --------- VISUALIZATION ------------------
        ///std::cout << _transformation << std::endl;
        ///
        ///pcl::visualization::PCLVisualizer viewer("result");
        ///
        ///Eigen::Affine3f pose;
        ///pose.matrix() = Eigen::Matrix4f::Identity();
        ///viewer.addCoordinateSystem(0.15, pose, "camera1");
        ///viewer.addPointCloud(_previousKf.cloud,"c1");
        ///viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c1");
        ///
        ///pose.matrix() = _transformation;
        ///viewer.addCoordinateSystem(0.15, pose, "camera2");
        ///pcl::PointCloud<PointType_> currentCloud, currentFeatureCloud;
        ///pcl::transformPointCloud(*_currentKf.cloud, currentCloud, _transformation);
        ///pcl::transformPointCloud(*_currentKf.featureCloud, currentFeatureCloud, _transformation);
        ///viewer.addPointCloud(currentCloud.makeShared(),"c2");
        ///viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c2");
        ///
        ///std::vector<cv::DMatch> inliers;
        ///ransacP2P.inliers(inliers);
        ///pcl::PointCloud<pcl::PointXYZRGB> ft1, ft2;
        ///unsigned pCounter = 0;
        ///for(auto inlier:inliers){
        ///    pcl::PointXYZRGB p1(255,0,0), p2(0,255,0);
        ///    p1.x = currentFeatureCloud.at(inlier.queryIdx).x;
        ///    p1.y = currentFeatureCloud.at(inlier.queryIdx).y;
        ///    p1.z = currentFeatureCloud.at(inlier.queryIdx).z;
        ///    ft1.push_back(p1);
        ///    p2.x = _previousKf.featureCloud->at(inlier.trainIdx).x;
        ///    p2.y = _previousKf.featureCloud->at(inlier.trainIdx).y;
        ///    p2.z = _previousKf.featureCloud->at(inlier.trainIdx).z;
        ///    ft2.push_back(p2);
        ///    viewer.addLine(  p1,p2, "line"+std::to_string(pCounter++));
        ///}
        ///viewer.addPointCloud(ft1.makeShared(),"fc1");
        ///viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "fc1");
        ///viewer.addPointCloud(ft2.makeShared(),"fc2");
        ///viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "fc2");
        ///
        ///
        ///while(!viewer.wasStopped()){
        ///   viewer.spinOnce(30);
        ///   std::this_thread::sleep_for(std::chrono::milliseconds(30));
        ///   cv::waitKey(3);
        ///}
        /// --------- VISUALIZATION ------------------

        return true;    //666 TODO check if transformation if valid and so on...
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::refineTransformation(const Keyframe<PointType_> &_previousKf, const Keyframe<PointType_> &_currentKf, Eigen::Matrix4f &_transformation){
        // Align
        pcl::IterativeClosestPointNonLinear<PointType_, PointType_> pcJoiner;
        pcJoiner.setTransformationEpsilon (mIcpMaxTransformationEpsilon);
        pcJoiner.setMaxCorrespondenceDistance (mIcpMaxCorrespondenceDistance);
        pcJoiner.setMaximumIterations(3);
		//pcJoiner.setUseReciprocalCorrespondences(true);

		pcl::registration::CorrespondenceRejectorOneToOne::Ptr corr_rej_one_to_one(new pcl::registration::CorrespondenceRejectorOneToOne);
		pcJoiner.addCorrespondenceRejector(corr_rej_one_to_one);

        pcl::PointCloud<PointType_> srcCloud = *_currentKf.cloud;
        pcl::PointCloud<PointType_> tgtCloud;


        pcJoiner.setInputTarget(_previousKf.cloud);
        pcl::PointCloud<PointType_> alignedCloud;
        Eigen::Matrix4f prevTransformation = _transformation;
        bool hasConvergedInSomeIteration = false;
        int i = 0;
        for (i = 0; i < mIcpMaxIterations; ++i){
            // Estimate

            pcJoiner.setInputSource(srcCloud.makeShared());
            pcJoiner.align(alignedCloud, _transformation);
            //accumulate transformation between each Iteration
            _transformation = pcJoiner.getFinalTransformation();
            if (pcJoiner.getFinalTransformation().hasNaN()) {
                std::cout << "--> MAP: Intermedial iteration of ICP throw transformation with NaN, skiping it and continuing iterations" << std::endl;
                break;
            }
            //if the difference between this transformation and the previous one
            //is smaller than the threshold, refine the process by reducing
            //the maximal correspondence distance
            if (fabs((pcJoiner.getLastIncrementalTransformation() - prevTransformation).sum()) < pcJoiner.getTransformationEpsilon()) {
                pcJoiner.setMaxCorrespondenceDistance(pcJoiner.getMaxCorrespondenceDistance()*0.9);
            }

            prevTransformation = pcJoiner.getLastIncrementalTransformation();
            hasConvergedInSomeIteration |= pcJoiner.hasConverged();

            if (pcJoiner.getMaxCorrespondenceDistance() < 0.001) {
                break;
            }
        }

        cout << "--> MAP: Exiting ICP iterations in " << i+1 << "/" << mIcpMaxIterations << endl;
        if (_transformation.hasNaN()) {
            cerr << "--> MAP:  ---> CRITICAL ERROR! Transformation has nans!!! <---" << endl;
            return false;
        }

        auto fittingScore = pcJoiner.getFitnessScore();
        //pcl::visualization::PCLVisualizer viewer("result");
        //viewer.addPointCloud(tgtCloud.makeShared(),"c1");
        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c1");
        //viewer.addPointCloud(alignedCloud.makeShared(),"c2");
        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "c2");
        //while(!viewer.wasStopped()){
        //   viewer.spinOnce(30);
        //   std::this_thread::sleep_for(std::chrono::milliseconds(30));
        //   cv::waitKey(3);
        //}
        return (pcJoiner.hasConverged() || hasConvergedInSomeIteration) && pcJoiner.getFitnessScore() < mIcpMaxFitnessScore;
    }


    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpEnabled(bool _enable) {
        mIcpEnabled = _enable;
    }


    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::icpEnabled() const {
        return mIcpEnabled;
    }


}

