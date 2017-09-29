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

#include <unordered_map>
#include <utils/Gui.h>

#include <pcl/common/transforms.h>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool BundleAdjuster<PointType_>::optimize() {
        #ifdef USE_CVSBA
            prepareData();

            std::vector<cv::Mat> ts, rs, intrinsics, coeffs;

            for(auto kf: mKeyframes){
                intrinsics.push_back(kf->intrinsic);
                coeffs.push_back(kf->coefficients);
                ts.push_back(cv::Mat(3,1,CV_32F, &kf->position[0]));
                auto rotation = kf->orientation.matrix();
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
            params.type = cvsba::Sba::MOTION;
            bundleAdjuster.setParams(params);

            assert(mScenePoints.size() == mScenePointsProjection[0].size());
            assert(mCovisibilityMatrix[0].size() == mScenePoints.size());
            assert(mCovisibilityMatrix.size() == mScenePointsProjection.size());
            assert(intrinsics.size() == mScenePointsProjection.size());
            assert(intrinsics.size() == coeffs.size());
            assert(intrinsics.size() == rs.size());
            assert(ts.size() == rs.size());

            bundleAdjuster.run(mScenePoints, mScenePointsProjection, mCovisibilityMatrix, intrinsics, rs, ts, coeffs);

//            rgbd::Gui::get()->clean(0);
//            pcl::PointCloud<pcl::PointXYZRGB> centroids;
//            for(auto kf:mKeyframes){
//                pcl::PointXYZRGB p(255,0,0);
//                p.x = kf->position[0];
//                p.y = kf->position[1];
//                p.z = kf->position[2];
//                centroids.push_back(p);
//            }

//            rgbd::Gui::get()->showCloud(centroids, "centroids", 5);

//            for(int i=1; i < centroids.size(); i++){
//                pcl::PointNormal p;
//                p.x = centroids[i-1].x;
//                p.y = centroids[i-1].y;
//                p.z = centroids[i-1].z;
//                p.normal_x = centroids[i].x - centroids[i-1].x;
//                p.normal_y = centroids[i].y - centroids[i-1].y;
//                p.normal_z = centroids[i].z - centroids[i-1].z;
//                rgbd::Gui::get()->drawArrow(p, "line"+std::to_string(i),255,0,0, 0);
//            }

            for(auto kf:mKeyframes){
                rgbd::Gui::get()->drawCoordinate(kf->pose, 0.05, 0);
            }

            Eigen::Matrix4f initPose = mKeyframes[0]->pose;
            Eigen::Matrix4f incPose;
            for(unsigned i = 0; i < ts.size(); i++){
                cv::Mat R;
                cv::Rodrigues(rs[i], R);

                Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();

                newPose(0,0) = R.at<double>(0,0);
                newPose(0,1) = R.at<double>(0,1);
                newPose(0,2) = R.at<double>(0,2);
                newPose(1,0) = R.at<double>(1,0);
                newPose(1,1) = R.at<double>(1,1);
                newPose(1,2) = R.at<double>(1,2);
                newPose(2,0) = R.at<double>(2,0);
                newPose(2,1) = R.at<double>(2,1);
                newPose(2,2) = R.at<double>(2,2);
                newPose(0,3) = ts[i].at<double>(0);
                newPose(1,3) = ts[i].at<double>(1);
                newPose(2,3) = ts[i].at<double>(2);

                //viewer.addCoordinateSystem(0.15, pose, "camera_" + std::to_string(i));

                if(i == 0){
                    incPose = newPose.inverse()*initPose;
                }

                //auto cloud = *mKeyframes[i]->cloud;
                //pcl::transformPointCloud(cloud, cloud, mKeyframes[i]->pose);
                //rgbd::Gui::get()->showCloud(cloud, "cloud0");

                newPose = incPose*newPose;

                mKeyframes[i]->position = newPose.block<3,1>(0,3);
                mKeyframes[i]->orientation = newPose.block<3,3>(0,0).matrix();
                mKeyframes[i]->pose = newPose;

                //cloud = *mKeyframes[i]->cloud;
                //pcl::transformPointCloud(cloud, cloud, mKeyframes[i]->pose);
                //rgbd::Gui::get()->showCloud(cloud, "cloud1");
                //
                //rgbd::Gui::get()->pause();
            }


            for(auto kf:mKeyframes){
                rgbd::Gui::get()->drawCoordinate(kf->pose, 0.1, 0);
            }


            rgbd::Gui::get()->pause();

//            pcl::PointCloud<pcl::PointXYZRGB> centroids2;
//            for(auto kf:mKeyframes){
//                pcl::PointXYZRGB p(0,255,0);
//                p.x = kf->position[0];
//                p.y = kf->position[1];
//                p.z = kf->position[2];
//                centroids2.push_back(p);
//            }

//            rgbd::Gui::get()->showCloud(centroids2, "centroids2", 5);

//            for(int i=1; i < centroids.size(); i++){
//                pcl::PointNormal p;
//                p.x = centroids2[i-1].x;
//                p.y = centroids2[i-1].y;
//                p.z = centroids2[i-1].z;
//                p.normal_x = centroids2[i].x - centroids2[i-1].x;
//                p.normal_y = centroids2[i].y - centroids2[i-1].y;
//                p.normal_z = centroids2[i].z - centroids2[i-1].z;
//                rgbd::Gui::get()->drawArrow(p, "line2"+std::to_string(i),0,255,0, 0);
//            }

//            rgbd::Gui::get()->pause();

//            for(unsigned i = 0; i < mKeyframes.size()-1; i++){
//                // Visualization ----
//                cv::Mat displayMatches;
//                cv::hconcat(mKeyframes[i]->left, mKeyframes[i+1]->left, displayMatches);
//                for(unsigned j = 0; j < mScenePointsProjection[i].size(); j++){
//                    if(!std::isnan(mScenePointsProjection[i][j].x) && !std::isnan(mScenePointsProjection[i+1][j].x)){
//                        cv::Point2i p1 = mScenePointsProjection[i][j];
//                        cv::Point2i p2 = mScenePointsProjection[i+1][j]; p2.x += mKeyframes[i]->left.cols;
//                        cv::circle(displayMatches, p1, 3, cv::Scalar(0,255,0),2);
//                        cv::circle(displayMatches, p2, 3, cv::Scalar(0,255,0),2);
//                        cv::line(displayMatches, p1, p2,  cv::Scalar(0,255,0),1);
//                    }
//                }
//                cv::imshow("displayMatches", displayMatches);
//                cv::waitKey();
//                // Visualization ----
//            }

            return true;
        #else
            std::cout << "CVSBA not installed! CANT PERFORM BA" << std::endl;
            return false;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::keyframes(std::vector<std::shared_ptr<Keyframe<PointType_>>> &_keyframes) {
        mKeyframes = _keyframes;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster<PointType_>::keyframes(typename std::vector<std::shared_ptr<Keyframe<PointType_>>>::iterator &_begin, typename std::vector<std::shared_ptr<Keyframe<PointType_>>>::iterator &_end) {
        mKeyframes.erase();
        mKeyframes.insert(mKeyframes.begin(), _begin, _end);
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
    std::vector<Keyframe<PointType_> , Eigen::aligned_allocator <Keyframe<PointType_>>> BundleAdjuster<PointType_>::keyframes() {
        return mKeyframes;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void BundleAdjuster<PointType_>::cleanData() {
        mScenePoints.clear();
        mScenePointsProjection.clear();
        mScenePointsProjection.resize(mKeyframes.size());
        mCovisibilityMatrix.clear();
        mCovisibilityMatrix.resize(mKeyframes.size());
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool BundleAdjuster<PointType_>::prepareData() {
        cleanData();

        std::unordered_map<int, int> idToIdx; // Map that maps from world id to data idx;
        // Allocate covisibility matrix for max size and then reduce at the end
        int maxSizeMat = 0;
        for(unsigned i = 0; i < mKeyframes.size(); i++){maxSizeMat += mKeyframes[i]->wordsReference.size();};

        int lastIdx = 0;
        for(unsigned kfIdx = 0; kfIdx < mKeyframes.size(); kfIdx++){
            mCovisibilityMatrix[kfIdx].resize(maxSizeMat, 0);
            for(unsigned wIdx = 0; wIdx < mKeyframes[kfIdx]->wordsReference.size(); wIdx++){
                auto &w = mKeyframes[kfIdx]->wordsReference[wIdx];
                if(w->frames.size() == 1){
                    continue;
                }

                auto idIter = idToIdx.find(w->id);
                if(idIter != idToIdx.end()){ // If word already added.
                    int id = idToIdx[w->id];
                    mCovisibilityMatrix[kfIdx][id] = 1;
                    mScenePointsProjection[kfIdx][id] = cv::Point2f(w->projections[mKeyframes[kfIdx]->id][0],
                                                                    w->projections[mKeyframes[kfIdx]->id][1]); // Check that all feature points are added as words.
                }else{  // If word not added yet
                    int id = lastIdx;
                    lastIdx++;
                    int wId = w->id;
                    idToIdx[wId] = id;
                    mCovisibilityMatrix[kfIdx][id] = 1;
                    mScenePoints.push_back(cv::Point3f(w->point[0],w->point[1],w->point[2]));
                    for(auto &v: mScenePointsProjection){
                        v.push_back(cv::Point2f(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()));
                    }
                    mScenePointsProjection[kfIdx][id] = cv::Point2f(w->projections[mKeyframes[kfIdx]->id][0],
                                                                    w->projections[mKeyframes[kfIdx]->id][1]);
                }
            }
        }
        for(auto &v: mCovisibilityMatrix){
            v.resize(idToIdx.size());
        }
    }

}
