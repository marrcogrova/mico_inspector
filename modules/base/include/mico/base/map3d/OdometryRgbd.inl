//---------------------------------------------------------------------------------------------------------------------
//  mico
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
    inline bool OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::init(cjson::Json _configFile) {
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
        ransacRefineIterations((int)_configFile["ransacRefineIterations"]);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::computeOdometry(std::shared_ptr<mico::DataFrame<PointType_>> _prevDf, std::shared_ptr<mico::DataFrame<PointType_>> _currentDf){
        // Filling new dataframe
        _currentDf->orientation = Eigen::Matrix3f::Identity();
        _currentDf->position = Eigen::Vector3f::Zero();

        if (_prevDf != nullptr && _currentDf != nullptr) {   
            if (!compute(_prevDf, _currentDf)) {
                this->error("ODOMETRY", "Failed computing odometry");
                return false;
            }
        }
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::computeOdometry(std::shared_ptr<mico::ClusterFrames<PointType_>> _prevCf, std::shared_ptr<mico::DataFrame<PointType_>> _currentDf){
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
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::compute(std::shared_ptr<mico::DataFrame<PointType_>> _prevDf, std::shared_ptr<mico::DataFrame<PointType_>> _currentDf){   
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        if (_prevDf != nullptr){
            if (!transformationBetweenFeatures<PointType_, DebugLevel_, OutInterface_>(_prevDf, _currentDf, transformation, mK_nearest_neighbors, mRansacMaxDistance, mRansacIterations, mRansacMinInliers, mFactorDescriptorDistance, mRansacRefineIterations)) {
                this->error("ODOMETRY", "Rejecting frame, cannot transform");
                return false; // reject keyframe.
            }

            if(mIcpEnabled){
                // Fine rotation.
                Eigen::Matrix4f icpTransformation = transformation;
                if(!icpAlignment<PointType_, DebugLevel_>(_currentDf->cloud,_prevDf->cloud,transformation,mIcpMaxIterations,
                                    mIcpMaxCorrespondenceDistance,
                                    0.707,
                                    0.3,
                                    1.0,
                                    1.0,
                                    mIcpMaxFitnessScore,
                                    mIcpVoxelDistance)){
                    this->error("ODOMETRY", "Cannot transform, ICP");
                }
                else{
                    transformation=icpTransformation;
                }
            }

            Eigen::Affine3f prevPose(_prevDf->pose);
            Eigen::Affine3f lastTransformation(transformation);
            Eigen::Affine3f currentPose = prevPose * lastTransformation;

            // Check transformation
            Eigen::Vector3f ea = transformation.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
            float angleThreshold = 20.0; ///180.0*M_PI;
            float distanceThreshold = 3.0;
            if ((abs(ea[0]) + abs(ea[1]) + abs(ea[2])) > angleThreshold || transformation.block<3, 1>(0, 3).norm() > distanceThreshold) {
                this->error("ODOMETRY", "Large transformation, DF not accepted");
                return false;
            }
            _currentDf->updatePose(currentPose.matrix());
            _currentDf->lastTransformation=lastTransformation;
        }
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline bool OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::compute(std::shared_ptr<mico::ClusterFrames<PointType_>> _prevCf, std::shared_ptr<mico::DataFrame<PointType_>> _currentDf){   
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        if (_prevCf != nullptr){
            if (!transformationBetweenFeatures<PointType_, DebugLevel_, OutInterface_>(_prevCf, _currentDf, transformation, mK_nearest_neighbors, mRansacMaxDistance, mRansacIterations, mRansacMinInliers, mFactorDescriptorDistance, mRansacRefineIterations)) {
                this->error("ODOMETRY", "Rejecting frame, cannot transform");
                return false; // reject keyframe.
            }

            if(mIcpEnabled){
                // Fine rotation.
                Eigen::Matrix4f icpTransformation = transformation;
                if(!icpAlignment<PointType_, DebugLevel_>(_currentDf->cloud,_prevCf->cloud,transformation,mIcpMaxIterations,
                                    mIcpMaxCorrespondenceDistance,
                                    0.707,
                                    0.3,
                                    1.0,
                                    1.0,
                                    mIcpMaxFitnessScore,
                                    mIcpVoxelDistance)){
                    this->error("ODOMETRY", "Cannot transform, ICP");
                }
                else{
                    transformation=icpTransformation;
                }
            }

            Eigen::Affine3f prevPose(_prevCf->pose);
            Eigen::Affine3f lastTransformation(transformation);
            Eigen::Affine3f currentPose = prevPose * lastTransformation;

            // Check transformation
            Eigen::Vector3f ea = transformation.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
            float angleThreshold = 20.0; ///180.0*M_PI;
            float distanceThreshold = 3.0;
            if ((abs(ea[0]) + abs(ea[1]) + abs(ea[2])) > angleThreshold || transformation.block<3, 1>(0, 3).norm() > distanceThreshold) {
                this->error("ODOMETRY", "Large transformation, DF not accepted");
                return false;
            }
            _currentDf->updatePose(currentPose.matrix());
            _currentDf->lastTransformation=lastTransformation;
        }
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::descriptorDistanceFactor(double _factor) {
        mFactorDescriptorDistance = _factor;
    }

    //---------------------------------------------------------------------------------------------------------------------

    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::ransacIterations(int _iterations) {
        mRansacIterations = _iterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::ransacMaxDistance(double _maxDistance) {
        mRansacMaxDistance = _maxDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::ransacMinInliers(int _minInliers) {
        mRansacMinInliers = _minInliers;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::ransacRefineIterations(int _refineIterations) {
        mRansacRefineIterations = _refineIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::k_nearest_neighbors(int _k_nearest_neighbors){
        mK_nearest_neighbors = _k_nearest_neighbors;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::icpEnabled(int _icpEnabled){
        mIcpEnabled=_icpEnabled;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::icpMaxCorrespondenceDistance(double _icpMaxCorrespondenceDistance){
        mIcpMaxCorrespondenceDistance=_icpMaxCorrespondenceDistance;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::icpMaxFitnessScore(double _icpMaxFitnessScore){
        mIcpMaxFitnessScore=_icpMaxFitnessScore;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::icpMaxIterations(int _icpMaxIterations){
        mIcpMaxIterations=_icpMaxIterations;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::icpMaxTransformationEpsilon(double _icpMaxTransformationEpsilon){
        mIcpMaxTransformationEpsilon=_icpMaxTransformationEpsilon;
    }
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_ >
    inline void OdometryRgbd<PointType_, DebugLevel_, OutInterface_>::icpVoxelDistance(double _icpVoxelDistance){
        mIcpVoxelDistance=_icpVoxelDistance;
    }
} // namespace mico 
