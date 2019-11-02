//---------------------------------------------------------------------------------------------------------------------
//  mico
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

namespace mico {
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
            mCameraIdToGraphId[_id] = vertexID;
            mGraphIdToCameraId[vertexID] = _id;

            // Camera vertex 
            g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
            v_se3->setId(vertexID);

            Eigen::Matrix4f poseInv = _pose.inverse();
            Eigen::Vector3d trans = poseInv.block<3,1>(0,3).cast<double>();
            Eigen::Quaterniond q(poseInv.block<3,3>(0,0).cast<double>());
            g2o::SE3Quat pose(q,trans);

            v_se3->setEstimate(pose);

            if (vertexID < 3)
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
            
            mPointIdToGraphId[_id] = mCurrentGraphID;
            mGraphIdToPointId[mCurrentGraphID] = _id;

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

            // this->status("BA_G2O","Projection camera  " + std::to_string(_idCamera)  +" ("+  std::to_string(mCameraIdToGraphId[_idCamera])
            //                             + ") to point " + std::to_string(_idPoint) +" ("+ std::to_string(mPointIdToGraphId[_idPoint]) +")");
            // 666 G2O does not handle distortion, there are two options, undistort points always outside or do it just here. But need to define it properly!
            //g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

            auto vertexPoint    = dynamic_cast<g2o::OptimizableGraph::Vertex*>(mOptimizer->vertices().find(mPointIdToGraphId[_idPoint])->second);
            auto vertexCamera   = dynamic_cast<g2o::OptimizableGraph::Vertex*>(mOptimizer->vertices().find(mCameraIdToGraphId[_idCamera])->second);

            // std::cout << "point: " << vertexPoint << ". ID: " << vertexPoint->id()  << std::endl;
            // std::cout << "camera: " << vertexCamera<< ". ID: " << vertexCamera->id() << std::endl;
            e->setVertex(0, vertexPoint);
            e->setVertex(1, vertexCamera);

            Eigen::Vector2d z(_projection.x, _projection.y);
            e->setMeasurement(z);
            e->information() = Eigen::Matrix2d::Identity();

            // Robust kernel for noise and outliers
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(0.5);  //666 tune val?
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

            mOptimizer->setVerbose(false); 
            
            std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
            
            // std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
            
            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
            );

            mOptimizer->setAlgorithm(solver);

            mPointIdToGraphId.clear();
            mCameraIdToGraphId.clear();
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
            // std::cout << "First opt: " << res;
            // std::cout << mOptimizer->edges().size() << std::endl;

            typedef std::pair<g2o::OptimizableGraph::Edge*, double> pairEdgeChi;
            std::vector<pairEdgeChi> edgeChiVals;
            int nBad = 0;
              int nGood = 0;
            std::vector<double> chiVals;
            for(auto &ep: mOptimizer->edges()){
                auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(ep);
                edgeChiVals.push_back(
                        pairEdgeChi(
                                    e,
                                    e->chi2()
                                    )
                                );
                chiVals.push_back(e->chi2());
            }

            std::sort(edgeChiVals.begin(), edgeChiVals.end(),[](pairEdgeChi &_a, pairEdgeChi &_b){
                return _a.second < _b.second;
            });

            // for(unsigned i = edgeChiVals.size()-1; i > edgeChiVals.size()*0.95; i--){
            for(unsigned i = 0; i < edgeChiVals.size(); i++){
                if(edgeChiVals[i].second > 6){
                    edgeChiVals[i].first->setLevel(1);
                    nBad++;
                    int graphPointId = edgeChiVals[i].first->vertex(0)->id();
                    int graphCameraId = edgeChiVals[i].first->vertex(1)->id();
                    mEdgeToRemove   [mGraphIdToCameraId[graphCameraId]  ]
                                    [mGraphIdToPointId[graphPointId]    ] = true;

                    this->warning("BA_G2O", "Mark to remove graph edge ("+std::to_string(graphCameraId)+", "+std::to_string(graphPointId)+") --> ("
                                                                        +std::to_string(mGraphIdToCameraId[graphCameraId])+", "+std::to_string(mGraphIdToPointId[graphPointId])+")");
                }
            }
            nGood = edgeChiVals.size() - nBad++;

            // std::cout << "nBad: " << nBad << ". nGood: " << nGood << std::endl;
            // std::cout << mOptimizer->edges().size() << std::endl;

            mOptimizer->initializeOptimization(0);

            // mOptimizer->save("g2o_graph.g2o2");
            mOptimizer->optimize(this->mBaIterations);
            // std::cout << ". Second Opt: " << res <<std::endl;
            return res;
        #else
            return false;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::recoverCamera(int _id, Eigen::Matrix4f &_pose, cv::Mat &_intrinsics, cv::Mat &_distcoeff){
        #ifdef USE_G2O
            int graphId = this->mCameraIdToGraphId[_id];

            g2o::VertexSE3Expmap * v_se3 = dynamic_cast< g2o::VertexSE3Expmap * > (mOptimizer->vertex(graphId));
            g2o::SE3Quat pose = v_se3->estimate();

            _pose = pose.to_homogeneous_matrix().cast<float>().inverse();
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::recoverPoint(int _id, Eigen::Vector3f &_position){
        #ifdef USE_G2O
            int graphId = this->mPointIdToGraphId[_id];

            g2o::VertexSBAPointXYZ * v_p= dynamic_cast< g2o::VertexSBAPointXYZ * > (mOptimizer->vertex(graphId));
            _position = v_p->estimate().cast<float>();
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjuster_g2o<PointType_, DebugLevel_, OutInterface_>::isProjectionEnabled(int _idCamera, int _idPoint){
        return !mEdgeToRemove[_idCamera][_idPoint];
    }
}