////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifdef USE_CVSBA
    #include <cvsba/cvsba.h>
#endif

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool BundleAdjuster<PointType_>::optimize() {
        #ifdef USE_CVSBA
            prepareData();

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

            bundleAdjuster.run(points, projections, visibility, intrinsics, rs, ts, coeffs);


            //
            //pcl::visualization::PCLVisualizer viewer("result");
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

               //viewer.addCoordinateSystem(0.15, pose, "camera_" + std::to_string(i));


               Eigen::Matrix3f r = pose.matrix().block<3,3>(0,0);
               pose.matrix().block<3,3>(0,0) = r.inverse();
               pose.matrix().block<3,1>(0,3) = -pose.matrix().block<3,1>(0,3);
               auto cloudKf = *mKeyframes[i].cloud;
               pcl::transformPointCloud(cloudKf, cloudKf, pose.matrix());

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
    inline void BundleAdjuster<PointType_>::keyframes(std::vector<Keyframe<PointType_> > &_keyframes) {

    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::keyframes(typename std::vector<Keyframe<PointType_> >::iterator &_begin, typename std::vector<Keyframe<PointType_> >::iterator &_end) {

    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double BundleAdjuster<PointType_>::minError       () const{
        return mBaMinError;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned BundleAdjuster<PointType_>::iterations     () const{
        return mBaIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned BundleAdjuster<PointType_>::minAparitions  () const{
        return mBaMinAparitions;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::minError         (double _error){
        mBaMinError = _error;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::iterations       (unsigned _iterations){
        mBaIterations = _iterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::minAparitions    (unsigned _aparitions){
        mBaMinAparitions = _aparitions;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    std::vector<Keyframe<PointType_> > BundleAdjuster<PointType_>::keyframes() {
        return mKeyframes;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool BundleAdjuster<PointType_>::prepareData() {
        // Init data
        mCovisibilityMatrix = cv::Mat(1,mKeyframes[0].featureCloud->size(), CV_32S);
        mSceneFeaturePoints.resize(mKeyframes[0].featureCloud->size());

        std::vector<int> descriptorToCovisibilityIndices(mKeyframes[0].featureCloud->size());
        for(unsigned i = 0; i < mKeyframes[0].featureCloud->size(); i++){
            auto  p = mKeyframes[0].featureCloud->at(i);
            mSceneFeaturePoints[i] = cv::Point3f(p.x, p.y, p.z);
            mCovisibilityMatrix.at<int>(0,i) = 1;
            descriptorToCovisibilityIndices[i] = i;

        }
        mDescriptorToCovisibilityIndices.push_back(descriptorToCovisibilityIndices);
        mSceneFeatureProjections.push_back(mKeyframes[0].featureProjections);


        // Complete data
        for(unsigned i = 1; i < mKeyframes.size(); i++){
            // Get indices of previous keyframe to the covisibility matrix.
            auto descriptorToCovisibilityPrev = mDescriptorToCovisibilityIndices.back();
            std::vector<int> descriptorToCovisibilityCurr(mKeyframes[i].featureCloud->size());

            // Extend covimSceneFeatureProjectionssibility matrix with matched points and add respective projections
            cv::Mat zeroRow = cv::Mat::zeros(1, mCovisibilityMatrix.cols, CV_32S);  // 666 TODO improve!
            vconcat(mCovisibilityMatrix, zeroRow,mCovisibilityMatrix);
            std::vector<cv::Point2f> projections(mCovisibilityMatrix.cols);
            for(auto match: mKeyframes[i].matchesPrev){
                mCovisibilityMatrix.at<int>(mCovisibilityMatrix.rows-1, descriptorToCovisibilityPrev[match.trainIdx]) = 1;
                descriptorToCovisibilityCurr[match.queryIdx] = descriptorToCovisibilityPrev[match.trainIdx];
                projections[descriptorToCovisibilityPrev[match.trainIdx]] = mKeyframes[i].featureProjections[match.queryIdx];
            }
            mSceneFeatureProjections.push_back(projections);

            // Extend new points and covisibility matrix and new projections
            int previousCols = mCovisibilityMatrix.cols;
            int noNewPoints = mKeyframes[i].featureCloud->size() - mKeyframes[i].matchesPrev.size();
            if(noNewPoints > 0){
                cv::Mat extensionCovisibility = cv::Mat::zeros(mCovisibilityMatrix.rows, noNewPoints, CV_32S);
                hconcat(mCovisibilityMatrix, extensionCovisibility, mCovisibilityMatrix);

                for(auto &vProj: mSceneFeatureProjections){
                    vProj.resize(previousCols+noNewPoints, cv::Point2f(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()));
                }
            }
            auto hasMatch = [](int idx, const std::vector<cv::DMatch> &_matches){
                bool hasMatch = false;
                for(auto match: _matches){
                    if(idx == match.queryIdx) hasMatch = true;
                }
                return hasMatch;
            };

            int newPointIndexCounter = 0;
            for(unsigned i = 0; i < mKeyframes[i].featureCloud->size(); i++){
                if(!hasMatch(i, mKeyframes[i].matchesPrev)){
                    mCovisibilityMatrix.at<int>(mCovisibilityMatrix.rows-1, previousCols+newPointIndexCounter) = 1;
                    descriptorToCovisibilityCurr[i] = previousCols+newPointIndexCounter;
                    mSceneFeatureProjections.back()[previousCols+newPointIndexCounter] = mKeyframes[i].featureProjections[i];
                    auto p = mKeyframes[i].featureCloud->at(i);
                    mSceneFeaturePoints.push_back(cv::Point3f(p.x, p.y, p.z));
                    newPointIndexCounter++;
                }
            }
            mDescriptorToCovisibilityIndices.push_back(descriptorToCovisibilityCurr);
        }
    }
}
