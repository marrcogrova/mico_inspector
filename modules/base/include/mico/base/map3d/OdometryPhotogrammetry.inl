//---------------------------------------------------------------------------------------------------------------------
//  MICO
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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


#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <mico/base/map3d/utils3d.h>



namespace mico {

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::init(cjson::Json _configFile) {
        descriptorDistanceFactor((double)_configFile["descriptorDistanceFactor"]);
        ransacIterations((int)_configFile["ransacIterations"]);
        ransacMaxDistance((double)_configFile["ransacMaxDistance"]); // 777 Test parameters
        ransacMinInliers((int)_configFile["ransacMinInliers"]);
        k_nearest_neighbors((int)_configFile["knearestneighbors"]);
        icpEnabled((int)_configFile["icpEnabled"]);
        icpMaxCorrespondenceDistance((double)_configFile["icpMaxCorrespondenceDistance"]);
        icpMaxFitnessScore((double)_configFile["icpMaxFitnessScore"]);
        icpMaxIterations((int)_configFile["icpMaxIterations"]);
        icpMaxTransformationEpsilon((double)_configFile["icpMaxTransformationEpsilon"]);
        icpVoxelDistance((double)_configFile["icpVoxelDistance"]);
        icpTimeOut((double)_configFile["icpTimeOut"]);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::computeOdometry(std::shared_ptr<mico::DataFrame<PointType_>> _prevDf, std::shared_ptr<mico::DataFrame<PointType_>> _currentDf){
    
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::computeOdometry(std::shared_ptr<mico::ClusterFrames<PointType_>> _prevCf, std::shared_ptr<mico::DataFrame<PointType_>> _currentDf){
        // Filling new dataframe
        _currentDf->orientation = Eigen::Matrix3f::Identity();
        _currentDf->position = Eigen::Vector3f::Zero();

        if (_prevCf != nullptr && _currentDf != nullptr) {   
            if (!compute(_prevCf, _currentDf)) {
                this->error("ODOMETRY", "Failed computing odometry");
                return false;
            }
        }
        return true;
    }


    //---------------------------------------------------------------------------------------------------------------------
    /* Correspondency DataFrame<->DataFrame */
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::compute(std::shared_ptr<mico::DataFrame<PointType_>> _prevDf, std::shared_ptr<mico::DataFrame<PointType_>> _currentDf){   
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    /* Correspondency ClusterFrame<->DataFrame */
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::compute(std::shared_ptr<mico::ClusterFrames<PointType_>> _prevCf, std::shared_ptr<mico::DataFrame<PointType_>> _currentDf){   

        Eigen::Matrix4f transformation=Eigen::Matrix4f::Identity();
        pcl::CorrespondencesPtr inliersCorresp(new pcl::Correspondences());
        
        if (_prevCf != nullptr){
            if (!transformationBetweenFeatures(_prevCf, _currentDf, transformation, mK_nearest_neighbors, 
                                                mRansacMaxDistance, mRansacIterations, mRansacMinInliers, 
                                                mFactorDescriptorDistance, mRansacRefineIterations, inliersCorresp)) {
                this->error("ODOMETRY", "Rejecting frame, cannot transform");
                return false; // reject keyframe.
            }


            // ------------- ICP ALGORITHM -------------
            if(mIcpEnabled){
                // Fine rotation.
                Eigen::Matrix4f icpTransformation = transformation;
                if(!icpPhotogram(_currentDf->featureCloud,_prevCf->featureCloud,
                                    icpTransformation,
                                    inliersCorresp,
                                    mIcpMaxIterations,
                                    mIcpMaxCorrespondenceDistance,
                                    0.707,  //_maxAngleDistance  (unused)
                                    0.3,    //_maxColorDistance (unused)
                                    1.0,    //_maxTranslation
                                    1.0,    //_maxRotation
                                    mIcpMaxFitnessScore,
                                    mIcpVoxelDistance,
                                    mIcpTimeOut) )
                {
                    this->error("ODOMETRY", "Cannot transform, ICP");
                }
                else{
                    transformation=icpTransformation;
                }
            }

            // Obtain current pose from tf estimated using ICP
            Eigen::Affine3f prevPose(_prevCf->pose);
            Eigen::Affine3f lastTransformation(transformation);
            Eigen::Affine3f currentPose = prevPose * lastTransformation;

            // Check transformation
            Eigen::Vector3f ea = transformation.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
            float angleThreshold = 5.0; // 180.0*M_PI;
            float distanceThreshold = 3.0; // Changed. (Initial value 1.0) 
            if ((abs(ea[0]) + abs(ea[1]) + abs(ea[2])) > angleThreshold || transformation.block<3, 1>(0, 3).norm() > distanceThreshold) {
                this->error("ODOMETRY", "Large transformation, DF not accepted");
                return false;
            } 
            _currentDf->updatePose(currentPose.matrix());
            _currentDf->lastTransformation=lastTransformation;
	
			return true;
        }else{
			return false;
		}
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::icpPhotogram(typename pcl::PointCloud<PointType_>::Ptr _source,
                                                                                            typename pcl::PointCloud<PointType_>::Ptr _target,
                                                                                            Eigen::Matrix4f &_transformation,
                                                                                            pcl::CorrespondencesPtr inliersCorrespondences,
                                                                                            int _iterations,
                                                                                            double _correspondenceDistance,
                                                                                            double _maxAngleDistance,
                                                                                            double _maxColorDistance,
                                                                                            double _maxTranslation,
                                                                                            double _maxRotation,
                                                                                            double _maxFitnessScore,
                                                                                            double _voxelGridSize,
                                                                                            double _timeout) {
        LoggableInterface<DebugLevel_, OutInterface_> logDealer;

        //auto t0 = std::chrono::high_resolution_clock::now();

        typename pcl::PointCloud<PointType_>::Ptr srcCloud(new pcl::PointCloud<PointType_>);
        std::vector<int> indices1,indices2;
        pcl::removeNaNFromPointCloud(*_source, *srcCloud, indices1);
        typename pcl::PointCloud<PointType_>::Ptr tgtCloud(new pcl::PointCloud<PointType_>);
        pcl::removeNaNFromPointCloud(*_target, *tgtCloud, indices2); 

        bool converged = false;
        int iters = 0;
        double corrDistance = _correspondenceDistance;

        // Using RANSAC inliers like first value of ICP correspondences 
        //*ptrCorr = *inliersCorrespondences;
        //std::cout << "Correspondences from RANSAC inliers: " << ptrCorr->size() << std::endl;

        pcl::CorrespondencesPtr ptrCorr(new pcl::Correspondences());
        while (/*!converged && */iters < _iterations /* && timeSpent < _timeout*/ ) {   
            iters++;
            typename pcl::PointCloud<PointType_>::Ptr cloudToAlign(new pcl::PointCloud<PointType_>);
            pcl::transformPointCloud(*srcCloud, *cloudToAlign, _transformation);
            std::vector<int> indices3;
            pcl::removeNaNFromPointCloud(*cloudToAlign, *cloudToAlign, indices3);

            // COMPUTE CORRESPONDENCES
            //if (iters > 1){
            pcl::registration::CorrespondenceEstimation<PointType_, PointType_> corresp_kdtree;
            corresp_kdtree.setInputSource(cloudToAlign);
            corresp_kdtree.setInputTarget(tgtCloud);
            corresp_kdtree.determineCorrespondences(*ptrCorr,corrDistance);

            //}
            if (ptrCorr->size() == 0) {
                logDealer.error("ICP_ALIGNEMENT", "Can't find any correspondence");
                break;
            }
            else {
                // Can be good idea test other type of rejector
                pcl::registration::CorrespondenceRejectorOneToOne::Ptr rejector(new pcl::registration::CorrespondenceRejectorOneToOne);
                rejector->setInputCorrespondences(ptrCorr);
                rejector->getCorrespondences(*ptrCorr);
                //std::cout << "Found " << ptrCorr->size() << " correspondences after one to one rejection" << std::endl;
            } 

            // Estimate transform
            Eigen::Matrix4f incTransform;
            //pcl::registration::TransformationEstimationPointToPlaneLLS<PointType_, PointType_, float> estimator;
            pcl::registration::TransformationEstimationSVD<PointType_, PointType_, float> estimator;          
            estimator.estimateRigidTransformation(*cloudToAlign, *tgtCloud, *ptrCorr,  incTransform);
            //std::cout << incTransform << std::endl;

            if (incTransform.hasNaN()) {
                logDealer.error("ICP_ALIGNEMENT", "Transformation of the cloud contains NaN");
                continue;
            }

            // COMPUTE SCORE
            double score = 0;
            for (unsigned j = 0; j < (*ptrCorr).size(); j++) {
                score += (*ptrCorr)[j].distance;
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
            corrDistance *= 0.9;    // To reduce the distance in each iteration

            //auto t1 = std::chrono::high_resolution_clock::now();
            //timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
        }   
        //std::cout << _transformation << std::endl;
        logDealer.status("ICP_PHOTOGRAMETRY", "Correspondences between current Df and previous Cf after "+ std::to_string(_iterations) + " iterations are: " + std::to_string( ptrCorr->size() ));

        return converged;
    }

 //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
	inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::matchDescriptorsBF(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers,double _mk_nearest_neighbors,double _mFactorDescriptorDistance){
		
		std::vector<std::vector<cv::DMatch>> matches12, matches21;
        cv::BFMatcher featureMatcher(cv::NORM_HAMMING,false); // (normType,crossCheck)
        featureMatcher.knnMatch(_des1, _des2, matches12,_mk_nearest_neighbors);
        featureMatcher.knnMatch(_des2, _des1, matches21,_mk_nearest_neighbors);
        // double max_dist = 0; double min_dist = 999999;
        //-- Quick calculation of max and min distances between keypoints   --- 666 LOOK FOR RATIO TEST AND IMPROVE THIS SHIT!
        // for( int i = 0; i < _des1.rows; i++ ) {
        //     for(int j = 0; j < matches12[i].size(); j++){
        //         double dist = matches12[i][j].distance;
        //         if( dist < min_dist ) min_dist = dist;
        //         if( dist > max_dist ) max_dist = dist;
        //     }
        // }

        // min_dist = min_dist==0 ? 2 : min_dist;


        std::vector<cv::DMatch> matches12fil, matches21fil; // 666 POSSIBLE OPTIMIZATION
        // RADIO TEST
        if(_mk_nearest_neighbors == 1){
            matches12fil = matches12[0];
            matches21fil = matches21[0];

			// symmetry test.
        	for( int i = 0; i < _des1.rows; i++ ){
        	    for(std::vector<cv::DMatch>::iterator it12 = matches12[i].begin(); it12 != matches12[i].end(); it12++){
        	        for( int j = 0; j < _des2.rows; j++ )
        	            for(std::vector<cv::DMatch>::iterator it21 = matches21[j].begin(); it21 != matches21[j].end(); it21++){
        	                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
        	                    if(it12->distance < _mFactorDescriptorDistance){
        	                        _inliers.push_back(*it12);
        	                        break;
        	                    }
        	                }
        	            }
        	    }

			}

        }else if(_mk_nearest_neighbors == 2){
            for(auto &matches: matches12){
                if(matches[0].distance < matches[1].distance * _mFactorDescriptorDistance){
                    matches12fil.push_back(matches[0]);
                }
            }

            for(auto &matches: matches21){
                if(matches[0].distance < matches[1].distance * _mFactorDescriptorDistance){
                    matches21fil.push_back(matches[0]);
                }
            }

			// symmetry test.
    		for(std::vector<cv::DMatch>::iterator it12 = matches12fil.begin(); it12 != matches12fil.end(); it12++){
    		    for(std::vector<cv::DMatch>::iterator it21 = matches21fil.begin(); it21 != matches21fil.end(); it21++){
    		        if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
    		            _inliers.push_back(*it12);
    		            break;
    		        }
    		    }
    		}
        }else{
            std::cout << "knn neighbors need to be 1 or 2" <<std::endl;
            return false;
        }

		if ((int)_inliers.size() > 0){
			std::cout << "-- symetric matches using BF matcher : " << _inliers.size() << std::endl;
			return true;
		}else{
			return false;
		}


	}


 //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
	inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::matchDescriptorsKDT(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers,double _mk_nearest_neighbors,double _mFactorDescriptorDistance){
        
        cv::Ptr<cv::DescriptorMatcher> matcherKDT = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(6,12, 2) ); // 12,20,2
		
		matcherKDT->add(_des2);
  		matcherKDT->train();
		std::vector<std::vector<cv::DMatch>> matches12, matches21;
		matcherKDT->knnMatch(_des1, _des2, matches12, 2 ); // _mk_nearest_neighbors
        matcherKDT->knnMatch(_des2, _des1, matches21, 2 ); // _mk_nearest_neighbors 
		//std::vector<cv::DMatch> matches;
		//matcherKDT -> match(_des1,_des2,matches);

		// Filter matches using the Lowe's ratio test
		const float ratio_thresh = 0.83f;
		std::vector<cv::DMatch> good_12,good_21;
		for (size_t i = 0; i < matches12.size(); i++){
			if (matches12[i][0].distance < ratio_thresh * matches12[i][1].distance){
				good_12.push_back(matches12[i][0]);
			}
		}
		for (size_t i = 0; i < matches21.size(); i++){
			if (matches21[i][0].distance < ratio_thresh * matches21[i][1].distance){
				good_21.push_back(matches21[i][0]);
			}
		}

		//std::cout << "There is : " << matches12.size() << " matches btw 1-2 and " << good_12.size() << " good \n";
		//std::cout << "There is : " << matches21.size() << " matches btw 2-1 and " << good_21.size() << " good \n";

        // symmetry test.
        for(std::vector<cv::DMatch>::iterator it12 = good_12.begin(); it12 != good_12.end(); it12++){
            for(std::vector<cv::DMatch>::iterator it21 = good_21.begin(); it21 != good_21.end(); it21++){
                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                    _inliers.push_back(*it12);
                    break;
                }
            }
        }
		if ((int)_inliers.size() > 0){
			std::cout << "-- symetric matches using FlannBased matcher : " << _inliers.size() << std::endl;
			return true;
		}else{
			return false;
		}
	}


 //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
	inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::matchPixels(std::shared_ptr<ClusterFrames<PointType_>> &_previousCf,
                                        std::shared_ptr<DataFrame<PointType_>> &_currentKf , std::vector<cv::DMatch> &_matches){


		//casting std::vector<cv::Point2f> to pcl::PointXY of feature projections
		pcl::PointCloud<pcl::PointXY>::Ptr curr_ftrs (new pcl::PointCloud<pcl::PointXY>);
  		curr_ftrs->width = _currentKf->featureProjections.size(); 
		curr_ftrs->height = 1;
  		curr_ftrs->points.resize(curr_ftrs->width * curr_ftrs->height);
  		for (size_t i = 0; i < curr_ftrs->points.size(); ++i){
  		  curr_ftrs->points[i].x = _currentKf->featureProjections[i].x;
  		  curr_ftrs->points[i].y = _currentKf->featureProjections[i].y;
  		}

		pcl::PointCloud<pcl::PointXY>::Ptr prev_ftrs (new pcl::PointCloud<pcl::PointXY>);
		prev_ftrs->width = _previousCf->featureProjections.size(); 
		prev_ftrs->height = 1;
  		prev_ftrs->points.resize(prev_ftrs->width * prev_ftrs->height);
  		for (size_t i = 0; i < prev_ftrs->points.size(); ++i){
  		  prev_ftrs->points[i].x = _previousCf->featureProjections[i].x;
  		  prev_ftrs->points[i].y = _previousCf->featureProjections[i].y;
  		}

		// Create PCL KD-Tree object and set as input previous image features
		pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXY>);
		kdtree->setInputCloud(prev_ftrs);

		// Search points in previous features belonging to radius for each point of current features
		std::vector<int> pointIdxRadiusSearch;
  		std::vector<float> pointRadiusSquaredDistance;

		cv::Ptr<cv::DescriptorMatcher> matcherFlann = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(6,12, 2) ); // 12,20,2
  		
		const float radius = 20.0f; // distance in px
		for (size_t i = 0; i < curr_ftrs->points.size(); i++){
			
			std::vector<cv::Point2f> CurrKeypointNeighborhood;
			std::vector<int> IdxNeightbourhood;

  			if ( kdtree->radiusSearch(curr_ftrs->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){

				cv::Mat KeypointDescriptor( _currentKf->featureDescriptors.row(i) );
				cv::Mat NeighborhoodDescriptors( 0 , pointIdxRadiusSearch.size() , CV_8U); 
				
				// Extract neighborhood keypoints and descriptors
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++){
					cv::Point2f neighbour;
  			    	neighbour.x = prev_ftrs->points[ pointIdxRadiusSearch[j] ].x;
  			        neighbour.y = prev_ftrs->points[ pointIdxRadiusSearch[j] ].y;
					CurrKeypointNeighborhood.push_back(neighbour);
					// Fill descriptors matrix
					NeighborhoodDescriptors.push_back( _previousCf->featureDescriptors.row( pointIdxRadiusSearch[j] ) );
					IdxNeightbourhood.push_back( pointIdxRadiusSearch[j] );

					
				}

				// Match current point descriptors with neighborhood
				matcherFlann->add(NeighborhoodDescriptors);
  				matcherFlann->train();
				
				std::vector<cv::DMatch> localMatches;
				matcherFlann->match(KeypointDescriptor, NeighborhoodDescriptors, localMatches );
				//std::vector<std::vector<cv::DMatch>> localMatches;
				//matcherFlann->knnMatch(KeypointDescriptor, NeighborhoodDescriptors, localMatches , 2);

				// std::cout << "Neighborhood size " << CurrKeypointNeighborhood.size() << std::endl;

				// std::cout << "query match idx: " << localMatches[0].queryIdx << " train match idx: " << localMatches[0].trainIdx << " image idx: " 
				// 		<< localMatches[0].imgIdx << " match distance: " << localMatches[0].distance << std::endl;
				// std::cout << "DATABASE query idx -> " << i << " train idx -> " << IdxNeightbourhood[localMatches[0].trainIdx] << std::endl;



				if (localMatches.size() > 0){
					//UnfilteredMatches.push_back(localMatches);
					
					// Used to print matches between images
					// std::vector<cv::KeyPoint> kp1,kp2;
					// cv::KeyPoint kp;
					// kp.pt.x = (float)_currentKf->featureProjections[i].x; 
					// kp.pt.y = (float)_currentKf->featureProjections[i].y;
					// kp1.push_back(kp);
					// cv::KeyPoint::convert(CurrKeypointNeighborhood,kp2);
					
					// cv::Mat image_matches;
					// cv::drawMatches(_currentKf->left, kp1, _previousCf->left, kp2 , localMatches, image_matches);
					// cv::imshow("Matches between current point and his neighborhood in previous frame ", image_matches);
					// cv::waitKey(0);

					localMatches[0].queryIdx = i;
					localMatches[0].trainIdx = IdxNeightbourhood[localMatches[0].trainIdx];
					//localMatches[0].imgIdx = _currentKf->id;

					_matches.push_back(localMatches[0]);
					matcherFlann->clear(); 
				}

  			}
		}
       
	    if (_matches.size() > 0){
			std::cout << "-- symetric matches using KDTree search and FLANN : " << _matches.size() << std::endl;
			return true;
		}else{
			return false;
		}
	}

 //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::transformationBetweenFeatures(std::shared_ptr<ClusterFrames<PointType_>> &_previousCf,
                                        std::shared_ptr<DataFrame<PointType_>> &_currentKf,
                                        Eigen::Matrix4f &_transformation,
                                        double _mk_nearest_neighbors,
                                        double _mRansacMaxDistance,
                                        int _mRansacIterations,
                                        double _mRansacMinInliers,
                                        double _mFactorDescriptorDistance,
                                        unsigned _mRansacRefineIterations,
                                        pcl::CorrespondencesPtr &inliersCorrespondences){

        LoggableInterface<DebugLevel_, OutInterface_> logDealer;

        if(_currentKf->multimatchesInliersKfs.find(_previousCf->id) !=  _currentKf->multimatchesInliersKfs.end()){
            // Match already computed
            logDealer.status("TRANSFORM_BETWEEN_FEATURES",  "Match already computed between frame: " + 
                                                            std::to_string(_currentKf->id) + " and cluster " + 
                                                            std::to_string(_previousCf->id));
            return true;
        }

		auto start = std::chrono::steady_clock::now();

		std::vector<cv::DMatch> matches;
		// matchPixels(_previousCf,_currentKf,matches);
		//matchPixelsExt(_previousCf,_currentKf,matches);
		matchDescriptorsKDT(_currentKf->featureDescriptors,_previousCf->featureDescriptors,matches,_mk_nearest_neighbors,_mFactorDescriptorDistance);
		//matchDescriptorsBF(_currentKf->featureDescriptors,_previousCf->featureDescriptors,matches,_mk_nearest_neighbors,_mFactorDescriptorDistance);

        auto end = std::chrono::steady_clock::now();
		std::cout << "Elapsed time MATCHES in milliseconds : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()<< " ms" << std::endl;


		// cv::Mat image_matches;
		// std::vector<cv::KeyPoint> kp1,kp2;
		// cv::KeyPoint::convert(_currentKf->featureProjections,kp1);
		// cv::KeyPoint::convert(_previousCf->featureProjections,kp2);
		// cv::drawMatches(_currentKf->left, kp1, _previousCf->left, kp2 , matches, image_matches);
    	// cv::imshow("Matches", image_matches);
		// cv::waitKey(0);


        std::vector<int> inliers;
        if(_mk_nearest_neighbors>1){
            typename pcl::PointCloud<PointType_>::Ptr duplicateCurrentKfFeatureCloud = _currentKf->featureCloud;
            *duplicateCurrentKfFeatureCloud += *_currentKf->featureCloud;
             mico::ransacAlignment<PointType_>( duplicateCurrentKfFeatureCloud,
                                                _previousCf->featureCloud,
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }else {
            mico::ransacAlignment<PointType_>(  _currentKf->featureCloud,
                                                _previousCf->featureCloud,
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }

        // cv::Mat display = _currentKf->left.clone();
        // for(auto &match:matches){
        //     cv::Point p2 = _currentKf->featureProjections[match.queryIdx];
        //     cv::circle(display, p2, 3, cv::Scalar(0,255,0), 1);
        // }
        // cv::imshow("FeatureMatcherRansac", display);
		// cv::waitKey(0);

        logDealer.status("TRANSFORM_BETWEEN_FEATURES", "Inliers between cf " + std::to_string(_previousCf->id) + " and kf " + 
                                                        std::to_string(_currentKf->id) + " = " + std::to_string(inliers.size()));
        if (inliers.size() >= _mRansacMinInliers) {
            _currentKf->multimatchesInliersCfs[_previousCf->id];
            _previousCf->multimatchesInliersKfs[_currentKf->id];
            int j = 0;
            for(unsigned int i = 0; i < inliers.size(); i++){
                while(matches[j].queryIdx != inliers[i]){
                    j++;
                }
                _currentKf->multimatchesInliersCfs[_previousCf->id].push_back(matches[j]);
                _previousCf->multimatchesInliersKfs[_currentKf->id].push_back(cv::DMatch(matches[j].trainIdx, matches[j].queryIdx, matches[j].distance));
                
                // Convert ransac inliers to PCL correspondences
                // matches[j].distance is diferent from correspondence.distance 
                pcl::Correspondence correspondence( matches[j].queryIdx, matches[j].trainIdx, matches[j].distance); // MUST BE SOLVED!!!

                //std::cout << matches[j].distance << std::endl;
				inliersCorrespondences->push_back(correspondence);
            }
            return true;
        }else{
            logDealer.error("TRANSFORM_BETWEEN_FEATURES", "Rejecting frame: Num Inliers <" + std::to_string(_mRansacMinInliers));
            return false;
        }
    }


    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::descriptorDistanceFactor(double _factor) {
        mFactorDescriptorDistance = _factor;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::k_nearest_neighbors(int _k_nearest_neighbors){
        mK_nearest_neighbors = _k_nearest_neighbors;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::icpEnabled(int _icpEnabled){
        mIcpEnabled=_icpEnabled;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::icpMaxCorrespondenceDistance(double _icpMaxCorrespondenceDistance){
        mIcpMaxCorrespondenceDistance=_icpMaxCorrespondenceDistance;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::icpMaxFitnessScore(double _icpMaxFitnessScore){
        mIcpMaxFitnessScore=_icpMaxFitnessScore;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::icpMaxIterations(int _icpMaxIterations){
        mIcpMaxIterations=_icpMaxIterations;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::icpMaxTransformationEpsilon(double _icpMaxTransformationEpsilon){
        mIcpMaxTransformationEpsilon=_icpMaxTransformationEpsilon;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::icpVoxelDistance(double _icpVoxelDistance){
        mIcpVoxelDistance=_icpVoxelDistance;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::icpTimeOut(double _icpTimeOut){
        mIcpTimeOut=_icpTimeOut;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::ransacIterations(int _iterations) {
        mRansacIterations = _iterations;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::ransacMaxDistance(double _maxDistance) {
        mRansacMaxDistance = _maxDistance;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::ransacMinInliers(int _minInliers) {
        mRansacMinInliers = _minInliers;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryPhotogrammetry<PointType_, DebugLevel_, OutInterface_>::ransacRefineIterations(int _refineIterations) {
        mRansacRefineIterations = _refineIterations;
    }


} // namespace mico
