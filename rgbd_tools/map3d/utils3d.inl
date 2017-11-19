////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <Eigen/Eigen>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>

namespace rgbd{
    template<typename PointType_>
    void ransacAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                         typename pcl::PointCloud<PointType_>::Ptr _target,
                         std::vector<cv::DMatch> &_matches,
                         Eigen::Matrix4f &_transformation,
                         std::vector<int> &_inliers,
                         double _maxRansacDistance,
                         int _ransacIterations,
                         unsigned _refineIterations) {

        std::vector<int> source_indices (_matches.size());   // 666 matchesPrev should be removed from kfs struct.
        std::vector<int> target_indices (_matches.size());

        // Copy the query-match indices
        for (int i = 0; i < (int)_matches.size(); ++i) {
            source_indices[i] = _matches[i].queryIdx;
            target_indices[i] = _matches[i].trainIdx;
        }

        typename pcl::SampleConsensusModelRegistration<PointType_>::Ptr model(new pcl::SampleConsensusModelRegistration<PointType_>(_source, source_indices));

        // Pass the target_indices
        model->setInputTarget (_target, target_indices);

        // Create a RANSAC model
        pcl::RandomSampleConsensus<PointType_> sac (model, _maxRansacDistance);

        sac.setMaxIterations(_ransacIterations);
        // Compute the set of inliers
        if(sac.computeModel()) {
            Eigen::VectorXf model_coefficients;

            sac.getInliers(_inliers);

            sac.getModelCoefficients (model_coefficients);

            // REFINEMENT STEP
            if (_refineIterations > 0) {
                double error_threshold = _maxRansacDistance;
                int refine_iterations = 0;
                bool inlier_changed = false, oscillating = false;
                std::vector<int> new_inliers, prev_inliers = _inliers;
                std::vector<size_t> inliers_sizes;
                Eigen::VectorXf new_model_coefficients = model_coefficients;
                do {
                    // Optimize the model coefficients
                    model->optimizeModelCoefficients (prev_inliers, new_model_coefficients, new_model_coefficients);
                    inliers_sizes.push_back (prev_inliers.size ());

                    // Select the new inliers based on the optimized coefficients and new threshold
                    model->selectWithinDistance (new_model_coefficients, error_threshold, new_inliers);

                    if (new_inliers.empty ()) {
                        ++refine_iterations;
                        if (refine_iterations >= _refineIterations) {
                            break;
                        }
                        continue;
                    }

                    // Estimate the variance and the new threshold
                    double variance = model->computeVariance ();
                    double refineSigma = 3.0;
                    error_threshold = std::min (_maxRansacDistance, refineSigma * sqrt(variance));

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
                } while (inlier_changed && ++refine_iterations < _refineIterations);

                std::swap (_inliers, new_inliers);
                model_coefficients = new_model_coefficients;
            }


            double covariance = model->computeVariance();
            // get best transformation
            Eigen::Matrix4f bestTransformation;
            bestTransformation.row (0) = model_coefficients.segment<4>(0);
            bestTransformation.row (1) = model_coefficients.segment<4>(4);
            bestTransformation.row (2) = model_coefficients.segment<4>(8);
            bestTransformation.row (3) = model_coefficients.segment<4>(12);
            _transformation = bestTransformation;
        }
    }

    template<typename PointType_>
    bool icpAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                      typename pcl::PointCloud<PointType_>::Ptr _target,
                      Eigen::Matrix4f &_transformation,
                      int _iterations,
                      double _correspondenceDistance,
                      double _maxAngleDistance,
                      double _maxColorDistance,
                      double _maxTranslation,
                      double _maxRotation,
                      double _maxFitnessScore) {
        pcl::PointCloud<PointType_> srcCloud;
        pcl::PointCloud<PointType_> tgtCloud;

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_target, tgtCloud, indices);
        pcl::removeNaNFromPointCloud(*_source, srcCloud, indices);

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
        double corrDistance = _correspondenceDistance;
        while (/*!converged &&*/ iters < _iterations) {
            iters++;
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
                std::cout << "Can't find any correspondences!" << std::endl;
                break;
            }
            else {
                pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejectorNormal(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
                rejectorNormal->setThreshold(_maxAngleDistance);
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
                    if (dist < _maxColorDistance) {
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
            Eigen::Quaternionf q(rot);
            Eigen::Quaternionf q0(Eigen::Matrix3f::Identity());
            double rotRes = fabs(q0.x() - q.x())+fabs(q0.z() - q.z())+fabs(q0.y() - q.y())+fabs(q0.w() - q.w());
            double transRes = fabs(incTransform.block<3, 1>(0, 3).sum());
            converged = (rotRes < _maxRotation &&  transRes < _maxTranslation) ? 1 : 0;

            //std::cout << "incT: " << transRes << ". incR: " << rotRes << ". Score: " << score << std::endl;
            converged = converged && (score < _maxFitnessScore);
            _transformation = incTransform*_transformation;
        }

        return converged;
    }

}
