////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <thread>
#include <chrono>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/gicp.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <iostream>

#include <rgbd_tools/map3d/msca.h>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline SceneRegistrator<PointType_>::SceneRegistrator(){
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::addKeyframe(std::shared_ptr<Keyframe<PointType_>> &_kf){
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        if(mLastKeyframe != nullptr){
            if((_kf->featureCloud == nullptr || _kf->featureCloud->size() ==0)) {
                auto t1 = std::chrono::high_resolution_clock::now();
                // Fine rotation.
                if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                    return false;   // reject keyframe.
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<"Refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;
            }else{
                // Match feature points.
                auto t0 = std::chrono::high_resolution_clock::now();
                // Compute initial rotation.
                if(!transformationBetweenFeatures( mLastKeyframe, _kf, transformation)){
                    return false;   // reject keyframe.
                }

                auto t1 = std::chrono::high_resolution_clock::now();

                if(mIcpEnabled){
                    // Fine rotation.
                    //std::cout << transformation << std::endl;
                    if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                        return false;   // reject keyframe.
                    }
                    //std::cout << transformation << std::endl;
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<	"\trough: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() <<
                                ", refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;

            }

            auto t0 = std::chrono::high_resolution_clock::now();
            Eigen::Affine3f prevPose(mLastKeyframe->pose);
            Eigen::Affine3f lastTransformation(transformation);
            // Compute current position.
            Eigen::Affine3f currentPose = prevPose*lastTransformation;

            // Check transformation
            Eigen::Vector3f ea = transformation.block<3,3>(0,0).eulerAngles(0, 1, 2);
            float angleThreshold = 20.0;///180.0*M_PI;
            float distanceThreshold = 0.3;
            if((abs(ea[0]) + abs(ea[1]) + abs(ea[2])) > angleThreshold || transformation.block<3,1>(0,3).norm() > distanceThreshold){
                std::cout << "Large transformation! not accepted KF" << std::endl;
                return false;
            }

            _kf->position = currentPose.translation();
            _kf->orientation = currentPose.rotation();
            _kf->pose = currentPose.matrix();
            auto t1 = std::chrono::high_resolution_clock::now();

            pcl::PointCloud<PointType_> cloud;
            pcl::transformPointCloudWithNormals(*_kf->cloud, cloud, _kf->pose);
            mMap += cloud;

            auto t2 = std::chrono::high_resolution_clock::now();

            pcl::VoxelGrid<PointType_> sor;
            sor.setInputCloud (mMap.makeShared());
            sor.setLeafSize (0.01f, 0.01f, 0.01f);
            sor.filter (mMap);

            auto t3 = std::chrono::high_resolution_clock::now();
            std::cout <<	"\tprepare T: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() <<
                            ", update map: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() <<
                            ", filter map: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << "-------------" <<std::endl;
            fillDictionary(_kf);
        }else{
//            // init dictionary with first cloud
//            for(unsigned idx = 0; idx < _kf->featureCloud->size(); idx++){
//                mWorldDictionary[idx] = std::shared_ptr<Word>(new Word);
//                mWorldDictionary[idx]->id       = idx;
//                mWorldDictionary[idx]->point    = {_kf->featureCloud->at(idx).x, _kf->featureCloud->at(idx).y, _kf->featureCloud->at(idx).z};
//                mWorldDictionary[idx]->frames   = {_kf->id};

//                _kf->wordsReference.push_back(mWorldDictionary[idx]);
//            }
            pcl::PointCloud<PointType_> cloud;
            pcl::transformPointCloudWithNormals(*_kf->cloud, cloud, _kf->pose);
            mMap += cloud;

            auto t2 = std::chrono::high_resolution_clock::now();

            pcl::VoxelGrid<PointType_> sor;
            sor.setInputCloud (mMap.makeShared());
            sor.setLeafSize (0.01f, 0.01f, 0.01f);
            sor.filter (mMap);
        }

        // Add keyframe to list.
        mKeyframes.push_back(_kf);
        mLastKeyframe = _kf;

        //const int cBaQueueSize = 5;
        //if(mKeyframesQueue.size() == cBaQueueSize){
        //    mBA.keyframes(mKeyframesQueue);
        //    mBA.optimize();
        //    mKeyframes.insert(mKeyframes.end(), mKeyframesQueue.begin(), mKeyframesQueue.end());
        //    mKeyframesQueue.clear();
        //
        //    mMap.clear();
        //    for(auto &kf:mKeyframes){
        //        pcl::PointCloud<PointType_> cloud;
        //        pcl::transformPointCloudWithNormals(*kf->cloud, cloud, kf->pose);
        //        mMap += cloud;
        //    }
        //
        //    pcl::VoxelGrid<PointType_> sor;
        //    sor.setInputCloud (mMap.makeShared());
        //    sor.setLeafSize (0.01f, 0.01f, 0.01f);
        //    sor.filter (mMap);
        //
        //    rgbd::Gui::get()->showCloud(mMap, "map",4,0);
        //}

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline std::vector<std::shared_ptr<Keyframe<PointType_>>>  SceneRegistrator<PointType_>::keyframes() const{
        return mKeyframes;
    }


    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    pcl::PointCloud<PointType_> SceneRegistrator<PointType_>::map() const{
        return mMap;
    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    std::shared_ptr<Keyframe<PointType_>> SceneRegistrator<PointType_>::lastFrame() const{
        return mLastKeyframe;
    }


    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::baMinError       () const{
        return mBA.minError();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned SceneRegistrator<PointType_>::baIterations     () const{
        return mBA.iterations();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned SceneRegistrator<PointType_>::baMinAparitions  () const{
        return mBA.minAparitions();
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
        mBA.minError(_error);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baIterations       (unsigned _iterations){
        mBA.iterations(_iterations);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baMinAparitions    (unsigned _aparitions){
        mBA.minAparitions(_aparitions);
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
    inline bool SceneRegistrator<PointType_>::matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers) {
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
        for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
            for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                    if(it12->distance <= min_dist*mFactorDescriptorDistance){
                        _inliers.push_back(*it12);
                    }
                    break;
                }
            }
        }
		return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool  SceneRegistrator<PointType_>::transformationBetweenFeatures(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
        matchDescriptors(_currentKf->featureDescriptors, _previousKf->featureDescriptors, _currentKf->matchesPrev);

        std::vector<int> source_indices (_currentKf->matchesPrev.size());
        std::vector<int> target_indices (_currentKf->matchesPrev.size());

        // Copy the query-match indices
        for (int i = 0; i < (int)_currentKf->matchesPrev.size(); ++i) {
            source_indices[i] = _currentKf->matchesPrev[i].queryIdx;
            target_indices[i] = _currentKf->matchesPrev[i].trainIdx;
        }

        typename pcl::SampleConsensusModelRegistration<PointType_>::Ptr model(new pcl::SampleConsensusModelRegistration<PointType_>(_currentKf->featureCloud, source_indices));

        // Pass the target_indices
        model->setInputTarget (_previousKf->featureCloud, target_indices);

        // Create a RANSAC model
        pcl::RandomSampleConsensus<PointType_> sac (model, mRansacMaxDistance);

        sac.setMaxIterations(mRansacIterations);
        // Compute the set of inliers
        if(sac.computeModel()) {
            std::vector<int> inliers;
            Eigen::VectorXf model_coefficients;

            sac.getInliers(inliers);

            sac.getModelCoefficients (model_coefficients);
/////////////////////////////
            int refineIterations=5;
            if (refineIterations > 0) {
                double error_threshold = mRansacMaxDistance;
                int refine_iterations = 0;
                bool inlier_changed = false, oscillating = false;
                std::vector<int> new_inliers, prev_inliers = inliers;
                std::vector<size_t> inliers_sizes;
                Eigen::VectorXf new_model_coefficients = model_coefficients;
                do {
                    // Optimize the model coefficients
                    model->optimizeModelCoefficients (prev_inliers, new_model_coefficients, new_model_coefficients);
                    inliers_sizes.push_back (prev_inliers.size ());

                    // Select the new inliers based on the optimized coefficients and new threshold
                    model->selectWithinDistance (new_model_coefficients, error_threshold, new_inliers);
                    //UDEBUG("RANSAC refineModel: Number of inliers found (before/after): %d/%d, with an error threshold of %f.",
                    //        (int)prev_inliers.size (), (int)new_inliers.size (), error_threshold);

                    if (new_inliers.empty ()) {
                        ++refine_iterations;
                        if (refine_iterations >= refineIterations) {
                            break;
                        }
                        continue;
                    }

                    // Estimate the variance and the new threshold
                    double variance = model->computeVariance ();
                    double refineSigma = 3.0;
                    error_threshold = std::min (mRansacMaxDistance, refineSigma * sqrt(variance));

                    inlier_changed = false;
                    std::swap (prev_inliers, new_inliers);

                    // If the number of inliers changed, then we are still optimizing
                    if (new_inliers.size () != prev_inliers.size ()) {
                        // Check if the number of inliers is oscillating in between two values
                        if (inliers_sizes.size () >= 4) {
                            if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
                            inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4]) {
                                oscillating = true;
                                break;
                            }
                        }
                        inlier_changed = true;
                        continue;
                    }

                    // Check the values of the inlier set
                    for (size_t i = 0; i < prev_inliers.size (); ++i) {
                        // If the value of the inliers changed, then we are still optimizing
                        if (prev_inliers[i] != new_inliers[i]){
                            inlier_changed = true;
                            break;
                        }
                    }
                }
                while (inlier_changed && ++refine_iterations < refineIterations);

                // If the new set of inliers is empty, we didn't do a good job refining
                if (new_inliers.empty ()){
                    //UWARN ("RANSAC refineModel: Refinement failed: got an empty set of inliers!");
                }

                if (oscillating){
                    //UDEBUG("RANSAC refineModel: Detected oscillations in the model refinement.");
                }

                std::swap (inliers, new_inliers);
                model_coefficients = new_model_coefficients;
            }
/////////////////////////////
            //cv::Mat display;
            //cv::hconcat(_previousKf->left, _currentKf->left, display);
            if (inliers.size() >= 3) {
                int j = 0;
                for(int i = 0; i < inliers.size(); i++){
                    while(_currentKf->matchesPrev[j].queryIdx != inliers[i]){
                        j++;
                    }
                    _currentKf->ransacInliers.push_back(_currentKf->matchesPrev[j]);

                    //cv::Point2i cvp = _currentKf->featureProjections[_currentKf->matchesPrev[j].queryIdx]; cvp.x += _previousKf->left.cols;
                    //cv::line(display, _previousKf->featureProjections[_currentKf->matchesPrev[j].trainIdx], cvp, cv::Scalar(0,255,0));
                }
                //cv::imshow("sadadad", display);
                //cv::waitKey();

                double covariance = model->computeVariance();


                // get best transformation
                Eigen::Matrix4f bestTransformation;
                bestTransformation.row (0) = model_coefficients.segment<4>(0);
                bestTransformation.row (1) = model_coefficients.segment<4>(4);
                bestTransformation.row (2) = model_coefficients.segment<4>(8);
                bestTransformation.row (3) = model_coefficients.segment<4>(12);
                _transformation = bestTransformation;

                return true;
            }
        }

        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::refineTransformation(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){

        pcl::PointCloud<PointType_> srcCloud;
        pcl::PointCloud<PointType_> tgtCloud;

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_previousKf->cloud, tgtCloud, indices);
        pcl::removeNaNFromPointCloud(*_currentKf->cloud, srcCloud, indices);

        pcl::VoxelGrid<PointType_> sor;
        sor.setInputCloud (srcCloud.makeShared());
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (srcCloud);
        sor.setInputCloud (tgtCloud.makeShared());
        sor.filter (tgtCloud);

        pcl::StatisticalOutlierRemoval<PointType_> sor2;
        sor2.setMeanK (50);
        sor2.setStddevMulThresh (1.0);
        sor2.setInputCloud (srcCloud.makeShared());
        sor2.filter (srcCloud);
        sor2.setInputCloud (tgtCloud.makeShared());
        sor2.filter (tgtCloud);

        bool converged = false;
        unsigned iters = 0;
        double corrDistance = mIcpMaxCorrespondenceDistance;
        while (/*!converged &&*/ iters < mIcpMaxIterations) {
            pcl::PointCloud<PointType_> cloudToAlign;
            //std::cout << _transformation << std::endl;
            pcl::transformPointCloudWithNormals(srcCloud, cloudToAlign, _transformation);

            // COMPUTE CORRESPONDENCES
            pcl::Correspondences correspondences;
            pcl::CorrespondencesPtr ptrCorr(new pcl::Correspondences);

            pcl::registration::CorrespondenceEstimation<PointType_, PointType_> corresp_kdtree;
            corresp_kdtree.setInputSource(cloudToAlign.makeShared());
            corresp_kdtree.setInputTarget(tgtCloud.makeShared());
            corresp_kdtree.determineCorrespondences(*ptrCorr, corrDistance);

            //std::cout << "Found " << ptrCorr->size() << " correspondences by distance" << std::endl;

            if (ptrCorr->size() == 0) {
                std::cout << "[MSCA] Can't find any correspondences!" << std::endl;
                break;
            }
            else {
                pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejectorNormal(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
                rejectorNormal->setThreshold(0.707);
                rejectorNormal->initializeDataContainer<PointType_, PointType_>();
                rejectorNormal->setInputSource<PointType_>(cloudToAlign.makeShared());
                rejectorNormal->setInputNormals<PointType_, PointType_>(cloudToAlign.makeShared());
                rejectorNormal->setInputTarget<PointType_>(tgtCloud.makeShared());
                rejectorNormal->setTargetNormals<PointType_, PointType_>(tgtCloud.makeShared());
                rejectorNormal->setInputCorrespondences(ptrCorr);
                rejectorNormal->getCorrespondences(*ptrCorr);
                //std::cout << "Found " << ptrCorr->size() << " correspondences after normal rejection" << std::endl;

                pcl::registration::CorrespondenceRejectorOneToOne::Ptr rejector(new pcl::registration::CorrespondenceRejectorOneToOne);
                rejector->setInputCorrespondences(ptrCorr);
                rejector->getCorrespondences(*ptrCorr);
                //std::cout << "Found " << ptrCorr->size() << " correspondences after one to one rejection" << std::endl;

                //pcl::CorrespondencesPtr ptrCorr2(new pcl::Correspondences);
                // Reject by color
                for (auto corr : *ptrCorr) {
                    // Measure distance
                    auto p1 = cloudToAlign.at(corr.index_query);
                    auto p2 = tgtCloud.at(corr.index_match);
                    double dist = sqrt(pow(p1.r - p2.r, 2) + pow(p1.g - p2.g, 2) + pow(p1.b - p2.b, 2));
                    dist /= sqrt(3) * 255;

                    // Add if approved
                    if (dist < 0.3) {
                        correspondences.push_back(corr);
                    }
                }

                //std::cout << "Found " << correspondences.size() << " correspondences after color rejection" << std::endl;
            }

            // Estimate transform
            pcl::registration::TransformationEstimationPointToPlaneLLS<PointType_, PointType_, float> estimator;
            //std::cout << _transformation << std::endl;
            Eigen::Matrix4f incTransform;
            estimator.estimateRigidTransformation(cloudToAlign, tgtCloud, correspondences,  incTransform);
            if (incTransform.hasNaN()) {
                std::cout << "[MSCA] Transformation of the cloud contains NaN!" << std::endl;
                continue;
            }

            // COMPUTE SCORE
            double score = 0;
            for (unsigned j = 0; j < correspondences.size(); j++) {
                score += correspondences[j].distance;
            }

            // CONVERGENCE
            Eigen::Matrix3f rot = incTransform.block<3, 3>(0, 0);
            //Eigen::Vector3f angles = rot.eulerAngles(0, 1, 2);
            //double rotRes = fabs(angles[0]) < M_PI ? angles[0] : angles[0] - truncf(angles[0]/M_PI)*M_PI +
            //                fabs(angles[1]) < M_PI ? angles[1] : angles[1] - truncf(angles[1]/M_PI)*M_PI +
            //                fabs(angles[2]) < M_PI ? angles[2] : angles[2] - truncf(angles[2]/M_PI)*M_PI;
            Eigen::Quaternionf q(rot);
            Eigen::Quaternionf q0(Eigen::Matrix3f::Identity());
            double rotRes = fabs(q0.x() - q.x())+fabs(q0.z() - q.z())+fabs(q0.y() - q.y())+fabs(q0.w() - q.w());
            double transRes = fabs(incTransform.block<3, 1>(0, 3).sum());
            converged = (rotRes < 0.005 &&  transRes < 0.001) ? 1 : 0;

            //std::cout << "incT: " << transRes << ". incR: " << rotRes << ". Score: " << score << std::endl;
            converged = converged && (score < mIcpMaxFitnessScore);
            _transformation = incTransform*_transformation;
            iters++;
            //if(transRes < 0.01){
            //    corrDistance *=0.9;
            //    corrDistance = corrDistance < 0.005?0.005:corrDistance;
            //}

            for(auto &p: cloudToAlign){
                p.r = 0;
                p.b = 0;
                p.g = 255;
            }

            //rgbd::Gui::get()->clean(1);
            //rgbd::Gui::get()->showCloud(tgtCloud,"tgtCloud", 3,1);
            //rgbd::Gui::get()->showCloud(cloudToAlign,"srcCloud", 3,1);
            //rgbd::Gui::get()->pause();
        }

        return converged;
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

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::fillDictionary(std::shared_ptr<Keyframe<PointType_>> &_kf){
        auto &prevKf = mLastKeyframe;
        pcl::PointCloud<PointType_> transCloud;
        pcl::transformPointCloud(*_kf->featureCloud, transCloud, _kf->pose);    // 666 Transform only chosen points not all.

        //cv::Mat display;
        //cv::hconcat(prevKf->left, _kf->left, display);
        for(unsigned inlierIdx = 0; inlierIdx < _kf->ransacInliers.size(); inlierIdx++){
            std::shared_ptr<Word> prevWord = nullptr;
            int inlierIdxInCurrent = _kf->ransacInliers[inlierIdx].queryIdx;
            int inlierIdxInPrev = _kf->ransacInliers[inlierIdx].trainIdx;
            for(auto &w: prevKf->wordsReference){
                if(w->idxInKf[prevKf->id] == inlierIdxInPrev){
                    prevWord = w;
                    break;
                }
            }
            if(prevWord){
                prevWord->frames.push_back(_kf->id);
                prevWord->projections[_kf->id] = {_kf->featureProjections[inlierIdxInCurrent].x, _kf->featureProjections[inlierIdxInCurrent].y};
                prevWord->idxInKf[_kf->id] = inlierIdxInCurrent;
                _kf->wordsReference.push_back(prevWord);
            }else{
                int wordId = mWorldDictionary.size();
                mWorldDictionary[wordId] = std::shared_ptr<Word>(new Word);
                mWorldDictionary[wordId]->id        = wordId;
                mWorldDictionary[wordId]->point     = {transCloud.at(inlierIdxInCurrent).x, transCloud.at(inlierIdxInCurrent).y, transCloud.at(inlierIdxInCurrent).z};
                mWorldDictionary[wordId]->frames    = {prevKf->id, _kf->id};
                mWorldDictionary[wordId]->projections[prevKf->id] = {prevKf->featureProjections[inlierIdxInPrev].x, prevKf->featureProjections[inlierIdxInPrev].y};
                mWorldDictionary[wordId]->projections[_kf->id] = {_kf->featureProjections[inlierIdxInCurrent].x, _kf->featureProjections[inlierIdxInCurrent].y};
                mWorldDictionary[wordId]->idxInKf[prevKf->id] = inlierIdxInPrev;
                mWorldDictionary[wordId]->idxInKf[_kf->id] = inlierIdxInCurrent;
                _kf->wordsReference.push_back(mWorldDictionary[wordId]);
                prevKf->wordsReference.push_back(mWorldDictionary[wordId]);

                //cv::Point2i cvp;
                //cvp.x = mWorldDictionary[wordId]->projections[_kf->id][0] + prevKf->left.cols;
                //cvp.y = mWorldDictionary[wordId]->projections[_kf->id][1];
                //cv::circle(display, cvp, 3, 3);
                //cv::Point2i cvp2;
                //cvp2.x = mWorldDictionary[wordId]->projections[prevKf->id][0];
                //cvp2.y = mWorldDictionary[wordId]->projections[prevKf->id][1];
                //cv::circle(display, cvp2, 3, 3);
                //cv::line(display, cvp2, cvp, cv::Scalar(0,255,0));
                //if(wordId < 10){
                //    std::cout << prevKf->id << ", " << wordId <<", "<< mWorldDictionary[wordId]->projections[prevKf->id][0] << ", " << mWorldDictionary[wordId]->projections[prevKf->id][1] <<std::endl;
                //    std::cout << _kf->id << ", "    << wordId <<", "<< mWorldDictionary[wordId]->projections[_kf->id][0] << ", " << mWorldDictionary[wordId]->projections[_kf->id][1] << std::endl;
                //}
            }
        }
        //cv::imshow("sadadad", display);
        //cv::waitKey();
    }


}

