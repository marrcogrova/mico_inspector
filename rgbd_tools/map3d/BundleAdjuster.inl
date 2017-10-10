////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include "cvsba/cvsba.h"

#include <unordered_map>
#include <utils/Gui.h>

#include <pcl/common/transforms.h>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool BundleAdjuster<PointType_>::optimize() {
        prepareData();

        std::vector<cv::Mat> listTranslations, listRotations, listIntrinsics, listCoeffs;

        for(auto kf: mKeyframes){
            cv::Mat intrinsics, coeffs;
            kf->intrinsic.convertTo(intrinsics, CV_64F);
            kf->coefficients.convertTo(coeffs, CV_64F);

            listIntrinsics.push_back(intrinsics.clone());
            listCoeffs.push_back(coeffs.clone());

            cv::Mat cvTrans(3,1,CV_64F);
            cvTrans.at<double>(0) = kf->position[0];
            cvTrans.at<double>(1) = kf->position[1];
            cvTrans.at<double>(2) = kf->position[2];
            listTranslations.push_back(cvTrans.clone());

            auto rotation = kf->orientation.matrix();
            cv::Mat cvRotation(3,3,CV_64F);
            cvRotation.at<double>(0,0) = rotation(0,0);
            cvRotation.at<double>(0,1) = rotation(0,1);
            cvRotation.at<double>(0,2) = rotation(0,2);
            cvRotation.at<double>(1,0) = rotation(1,0);
            cvRotation.at<double>(1,1) = rotation(1,1);
            cvRotation.at<double>(1,2) = rotation(1,2);
            cvRotation.at<double>(2,0) = rotation(2,0);
            cvRotation.at<double>(2,1) = rotation(2,1);
            cvRotation.at<double>(2,2) = rotation(2,2);

            //rgbd::Gui::get()->pause();
            listRotations.push_back(cvRotation.clone());
        }

        // Initialize cvSBA and perform bundle adjustment.
        cvsba::Sba bundleAdjuster;
        cvsba::Sba::Params params;
        params.verbose = true;
        params.iterations = mBaIterations;
        params.minError = mBaMinError;
        //params.type = cvsba::Sba::MOTION;
        bundleAdjuster.setParams(params);

        assert(mScenePoints.size() == mScenePointsProjection[0].size());
        assert(mCovisibilityMatrix[0].size() == mScenePoints.size());
        assert(mCovisibilityMatrix.size() == mScenePointsProjection.size());
        assert(listIntrinsics.size() == mScenePointsProjection.size());
        assert(listIntrinsics.size() == listCoeffs.size());
        assert(listIntrinsics.size() == listRotations.size());
        assert(listTranslations.size() == listRotations.size());

        for(auto kf:mKeyframes){
            rgbd::Gui::get()->drawCoordinate(kf->pose, 0.05, 0);
        }

        pcl::PointCloud<pcl::PointXYZRGB> baCloud;
        for(auto &point: mScenePoints){
            pcl::PointXYZRGB p(0,0,255);
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            baCloud.push_back(p);
        }
        rgbd::Gui::get()->showCloud(baCloud, "bacloud", 3, 0);

        bundleAdjuster.run(mScenePoints, mScenePointsProjection, mCovisibilityMatrix, listIntrinsics, listRotations, listTranslations, listCoeffs);

        pcl::PointCloud<pcl::PointXYZRGB> baCloud2;
        for(auto &point: mScenePoints){
            pcl::PointXYZRGB p(0,255,0);
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            baCloud2.push_back(p);
        }
        rgbd::Gui::get()->showCloud(baCloud2, "bacloud2", 3, 0);

        Eigen::Matrix4f initPose = mKeyframes[0]->pose;
        Eigen::Matrix4f incPose;
        for(unsigned i = 0; i < listTranslations.size(); i++){
            cv::Mat R = listRotations[i];
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
            //rgbd::Gui::get()->pause();

            newPose(0,3) = listTranslations[i].at<double>(0);
            newPose(1,3) = listTranslations[i].at<double>(1);
            newPose(2,3) = listTranslations[i].at<double>(2);

            //std::cout << mKeyframes[i]->pose << std::endl;
            //std::cout << newPose << std::endl;

            if(i == 0){
                incPose = newPose.inverse()*initPose;
            }
            //newPose = incPose*newPose;

            mKeyframes[i]->position = newPose.block<3,1>(0,3);
            mKeyframes[i]->orientation = newPose.block<3,3>(0,0).matrix();
            mKeyframes[i]->pose = newPose;

            rgbd::Gui::get()->drawCoordinate(newPose, 0.1, 0);
        }

        for(unsigned i = 0; i < mKeyframes.size()-1; i++){
            // Visualization ----
            cv::Mat displayMatches;
            cv::Mat displayProj = mKeyframes[i]->left;
            cv::hconcat(mKeyframes[i]->left, mKeyframes[i+1]->left, displayMatches);

            std::cout << mKeyframes[i]->pose << std::endl;

            pcl::PointCloud<pcl::PointXYZ> relativeCloud;
            for(unsigned pidx =0; pidx < mScenePoints.size(); pidx++){
                Eigen::Vector4f relativePosition (mScenePoints[pidx].x, mScenePoints[pidx].y, mScenePoints[pidx].z,1);
                relativePosition = initPose.inverse()*mKeyframes[i]->pose*relativePosition;
                pcl::PointXYZ p(relativePosition(0), relativePosition(1), relativePosition(2));
                relativeCloud.push_back(p);
            }
            rgbd::Gui::get()->showCloud(relativeCloud, "relativeCloud", 3);
            rgbd::Gui::get()->pause();
            rgbd::Gui::get()->clean("relativeCloud");

            std::vector<cv::Point2d> imagePoints;
            cv::projectPoints(mScenePoints, listRotations[i].clone(), listTranslations[i], mKeyframes[i]->intrinsic, mKeyframes[i]->coefficients, imagePoints);
            for(unsigned j = 0; j < mScenePointsProjection[i].size(); j++){
                if(!std::isnan(mScenePointsProjection[i][j].x) && !std::isnan(mScenePointsProjection[i+1][j].x)){
                    cv::Point2i p1 = mScenePointsProjection[i][j];
                    cv::Point2i p2 = mScenePointsProjection[i+1][j]; p2.x += mKeyframes[i]->left.cols;
                    cv::circle(displayMatches, p1, 3, cv::Scalar(0,255,0),2);
                    cv::circle(displayMatches, p2, 3, cv::Scalar(0,255,0),2);
                    cv::line(displayMatches, p1, p2,  cv::Scalar(0,255,0),1);

                    // -->>> 666 Take care of CS for projections, there is a drift but ignore it by now
                    //if( j < 10 ){
                        cv::circle(displayProj, p1, 3, cv::Scalar(0,0,255),3);

                        Eigen::Vector4f relativePosition (mScenePoints[j].x, mScenePoints[j].y, mScenePoints[j].z,1);
                        relativePosition = mKeyframes[i]->pose*relativePosition;

                        cv::Mat intrinsicMatrix = mKeyframes[i]->intrinsic;
                        float cx = intrinsicMatrix.at<float>(0,2);
                        float cy = intrinsicMatrix.at<float>(1,2);
                        float fx = intrinsicMatrix.at<float>(0,0);
                        float fy = intrinsicMatrix.at<float>(1,1);
                        int x = relativePosition(0)/relativePosition(2)*fx + cx;
                        int y = relativePosition(1)/relativePosition(2)*fy + cy;
                        cv::Point2i p1b(x,y);
                        cv::circle(displayProj, p1b, 3, cv::Scalar(255,0,0),2);
                        //std::cout << p1.x << ", " << p1.y << "; " << p1a.x << ", " << p1a.y << std::endl;
                        cv::Point2i p1a = imagePoints[j];
                        cv::circle(displayProj, p1a, 3, cv::Scalar(0,255,0),2);
                    //}

                    //if(j < 10){
                    //    std::cout << i      << ", " << j <<", " <<  p1.x << ", " << p1.y <<std::endl;
                    //    std::cout << i+1    << ", " << j <<", " << p2.x - mKeyframes[i]->left.cols<< ", " << p2.y << std::endl;
                    //}
                }
            }
            cv::imshow("displayMatches", displayMatches);
            cv::imshow("displayProj", displayProj);
            cv::waitKey();
            // Visualization ----
        }

        return true;
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
                    mScenePointsProjection[kfIdx][id] = cv::Point2d(w->projections[mKeyframes[kfIdx]->id][0],
                                                                    w->projections[mKeyframes[kfIdx]->id][1]); // Check that all feature points are added as words.
                }else{  // If word not added yet
                    int id = lastIdx;
                    lastIdx++;
                    int wId = w->id;
                    idToIdx[wId] = id;
                    mCovisibilityMatrix[kfIdx][id] = 1;
                    mScenePoints.push_back(cv::Point3d(w->point[0],w->point[1],w->point[2]));
                    for(auto &v: mScenePointsProjection){
                        v.push_back(cv::Point2d(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()));
                    }
                    mScenePointsProjection[kfIdx][id] = cv::Point2d(w->projections[mKeyframes[kfIdx]->id][0],
                                                                    w->projections[mKeyframes[kfIdx]->id][1]);
                }
            }
        }
        for(auto &v: mCovisibilityMatrix){
            v.resize(idToIdx.size());
        }
    }

}
