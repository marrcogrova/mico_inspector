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
}
