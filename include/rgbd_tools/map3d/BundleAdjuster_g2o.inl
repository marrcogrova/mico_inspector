//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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

#include <rgbd_tools/utils/Graph2d.h>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::BundleAdjuster_g2o() {
        // Init optimizer
        #ifdef USE_G2O
           

        #endif
    }


    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::appendCamera(int _id, Eigen::Matrix4f _pose, cv::Mat _intrinsics, cv::Mat _distcoeff){
        #ifdef USE_G2O
            if(_id == 0){ // Assuming IDs starts always from 0
                double focal_length = _intrinsics.at<double>(0,0);
                Eigen::Vector2d principal_point(_intrinsics.at<double>(0,2), 
                                                _intrinsics.at<double>(1,2)    );
    
                g2o::CameraParameters * cam_params = new g2o::CameraParameters(focal_length, principal_point, 0.);
                cam_params->setId(0);

                assert(mOptimizer->addParameter(cam_params));
            }

            // this->status("BA_G2O","Camera " + std::to_string(_id) + " as vertex " + std::to_string(mCurrentGraphID));

            int vertexID = mCurrentGraphID;
            mCameraId2GraphId[_id] = vertexID;

            // Camera vertex 
            g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
            v_se3->setId(vertexID);


            Eigen::Vector3d trans = _pose.block<3,1>(0,3).cast<double>();
            Eigen::Quaterniond q(_pose.block<3,3>(0,0).cast<double>());
            g2o::SE3Quat pose(q,trans);

            v_se3->setEstimate(pose);

            if (vertexID < 1)
                v_se3->setFixed(true);

            mOptimizer->addVertex(v_se3);
            mCurrentGraphID++;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::appendPoint(int _id, Eigen::Vector3f _position){
        #ifdef USE_G2O
            // this->status("BA_G2O","Point " + std::to_string(_id) + " as vertex " + std::to_string(mCurrentGraphID));
            
            mPointId2GraphId[_id] = mCurrentGraphID;

            g2o::VertexSBAPointXYZ * v_p = new g2o::VertexSBAPointXYZ();

            v_p->setId(mCurrentGraphID);
            v_p->setMarginalized(true);
            v_p->setEstimate(_position.cast<double>());

            mOptimizer->addVertex(v_p);

            mCurrentGraphID++;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::appendProjection(int _idCamera, int _idPoint, cv::Point2f _projection, cv::Mat _intrinsics, cv::Mat _distcoeff){
        #ifdef USE_G2O

            // this->status("BA_G2O","Projection camera  " + std::to_string(_idCamera)  +" ("+  std::to_string(mCameraId2GraphId[_idCamera])
            //                             + ") to point " + std::to_string(_idPoint) +" ("+ std::to_string(mPointId2GraphId[_idPoint]) +")");
            // 666 G2O does not handle distortion, there are two options, undistort points always outside or do it just here. But need to define it properly!
            //g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

            auto vertexPoint    = dynamic_cast<g2o::OptimizableGraph::Vertex*>(mOptimizer->vertices().find(mPointId2GraphId[_idPoint])->second);
            auto vertexCamera   = dynamic_cast<g2o::OptimizableGraph::Vertex*>(mOptimizer->vertices().find(mCameraId2GraphId[_idCamera])->second);

            // std::cout << "point: " << vertexPoint << ". ID: " << vertexPoint->id()  << std::endl;
            // std::cout << "camera: " << vertexCamera<< ". ID: " << vertexCamera->id() << std::endl;
            e->setVertex(0, vertexPoint);
            e->setVertex(1, vertexCamera);

            Eigen::Vector2d z(_projection.x, _projection.y);
            e->setMeasurement(z);
            e->information() = Eigen::Matrix2d::Identity();

            // Robust kernel for noise and outliers
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            
            //e->setParameterId(0, 0);    // Set camera params
            
            e->fx =  _intrinsics.at<double>(0,0);
            e->fy =  _intrinsics.at<double>(1,1);
            e->cx =  _intrinsics.at<double>(0,2);
            e->cy =  _intrinsics.at<double>(1,2);

            e->setLevel(0);
            
            mOptimizer->addEdge(e);
            // mEdgesList.push_back(e);
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::reserveData(int _cameras, int _words){

    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::fitSize(int _cameras, int _words){

    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::cleanData(){
        #ifdef USE_G2O
            if(mOptimizer != nullptr)
                delete mOptimizer;
            
            mOptimizer = new g2o::SparseOptimizer;

            mOptimizer->setVerbose(true);
            
            std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
            
            // std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
            
            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
            );

            mOptimizer->setAlgorithm(solver);

            mPointId2GraphId.clear();
            mCameraId2GraphId.clear();
            mCurrentGraphID = 0;
            mEdgesList.clear();

        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::checkData(){

    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::doOptimize(){
        #ifdef USE_G2O
            mOptimizer->initializeOptimization(0);

            // std::cout << mOptimizer->edges().size() << std::endl;
            mOptimizer->save("g2o_graph.g2o");
            bool res = mOptimizer->optimize(this->mBaIterations);
            std::cout << mOptimizer->edges().size() << std::endl;

            // std::vector<double> chiVals;
            // int nBad = 0;
            // int nGood = 0;
            // for(auto &ep: mOptimizer->edges()){
            //     auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(ep);
            //     chiVals.push_back(e->chi2());
            //     if(e->chi2() > 50000){
            //         e->setLevel(1);
            //         nBad++;
            //     }else{
            //         e->setLevel(0);
            //         nGood++;
            //     }
            // }

            // std::cout << "nBad: " << nBad << ". nGood: " << nGood << std::endl;

            // std::cout << mOptimizer->edges().size() << std::endl;

            // Graph2d graph("chi vals");
            // graph.draw(chiVals, 255,0,0, Graph2d::eDrawType::Lines);
            // graph.show();
            // cv::waitKey();

            // mOptimizer->initializeOptimization(0);

            // mOptimizer->save("g2o_graph.g2o2");
            // res &= mOptimizer->optimize(this->mBaIterations);

            return res;
        #else
            return false;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::recoverCameras(){
        #ifdef USE_G2O
            for(unsigned idx = 0; idx < this->mCameraId2GraphId.size(); idx++){
                int id = this->mCameraId2GraphId[idx];

                g2o::VertexSE3Expmap * v_se3 = dynamic_cast< g2o::VertexSE3Expmap * > (mOptimizer->vertex(id));
                //if(v_se3 != nullptr){         //Removed condition   666 THIS IS GOOD!, isn't it?? 666 666 666
                    //std::cout << "Pose of df: " << frameId.first << std::endl << mClusterframe->poses[frameId.first] << std::endl;
                    g2o::SE3Quat pose;
                    pose = v_se3->estimate();
                        
                    auto cluster = this->mClusterFrames[id]; 

                    Eigen::Matrix4f newPose = pose.to_homogeneous_matrix().cast<float>();
                    cluster->bestDataframePtr()->updatePose(newPose);
                    Eigen::Matrix4f offsetCluster = cluster->bestDataframePtr()->pose.inverse()*newPose;

                    for(auto &df : cluster->dataframes){
                        if(df.second->id != cluster->bestDataframe){
                            Eigen::Matrix4f updatedPose = offsetCluster*df.second->pose;
                            df.second->updatePose(updatedPose);
                        }
                    }
                //}
            }
        #endif
    }


    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::recoverPoints(){
        #ifdef USE_G2O
            for(unsigned idx = 0; idx < this->mPointId2GraphId.size(); idx++){
                int graphId = this->mPointId2GraphId[idx];
                int wordId = this->mWordIdxToId[idx];

                g2o::VertexSBAPointXYZ * v_p= dynamic_cast< g2o::VertexSBAPointXYZ * > (mOptimizer->vertex(graphId));
                Eigen::Vector3d p_pos = v_p->estimate();
                this->mGlobalUsedWordsRef[wordId]->point = {
                    (float) p_pos[0],
                    (float) p_pos[1],
                    (float) p_pos[2]
                };
                this->mGlobalUsedWordsRef[wordId]->optimized = true;
            }
        #endif
    }
}