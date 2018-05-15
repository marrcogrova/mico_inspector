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

#ifdef USE_G2O

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline BundleAdjuster_g2o<PointType_>::BundleAdjuster_g2o() {
        // Init optimizer
        mOptimizer.setVerbose(true);

        std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>  linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();        
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
        );

        mOptimizer.setAlgorithm(solver);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool BundleAdjuster_g2o<PointType_>::optimize() {
        mOptimizer.initializeOptimization();
        std::cout << "Performing full BA:" << std::endl;
        auto  result = mOptimizer.optimize(mBaIterations);

        // Recover poses.
        for(auto &kfId: kfId2GraphId){
            g2o::VertexSE3Expmap * v_se3 = dynamic_cast< g2o::VertexSE3Expmap * > (mOptimizer.vertex(kfId.second));
            if(v_se3 != 0){
                g2o::SE3Quat pose;
                pose = v_se3->estimate();
                mDataframes[kfId.first]->position = pose.translation().cast<float>();
                mDataframes[kfId.first]->orientation = pose.rotation().cast<float>();
                Eigen::Matrix4f poseEigen = Eigen::Matrix4f::Identity();
                poseEigen.block<3,3>(0,0) = mDataframes[kfId.first]->orientation.matrix();
                poseEigen.block<3,1>(0,3) = mDataframes[kfId.first]->position;
                mDataframes[kfId.first]->pose = poseEigen;
            }
        }

        // Recover words points.

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster_g2o<PointType_>::keyframes(std::vector<std::shared_ptr<DataFrame<PointType_>>> &_keyframes) {
        // Copy dataframes
        mDataframes = _keyframes;

        // 666 CLEAN OPTIMIZER???

        // Set camera parameters
        cv::Mat intrinsics = mDataframes[0]->intrinsic;
        double focal_length = intrinsics.at<float>(0,0);
        Eigen::Vector2d principal_point(intrinsics.at<float>(0,2), 
                                        intrinsics.at<float>(1,2)    );

        g2o::CameraParameters * cam_params = new g2o::CameraParameters (focal_length, principal_point, 0.);
        cam_params->setId(0);

        if (!mOptimizer.addParameter(cam_params)) {
            assert(false);
        }

        int grasphIdCounter = 0;
        kfId2GraphId.clear();
        wordId2GraphId.clear();

        std::unordered_map<int, bool> idsUsed;
        bool isFirst = true;
        // Register frames first
        for(auto &frame: mDataframes){
            //  Add pose
            Eigen::Vector3f position = frame->position;
            Eigen::Quaternionf orientation = frame->orientation;
            Eigen::Vector3d trans = position.cast<double>();
            Eigen::Quaterniond q = orientation.cast<double>();

            g2o::SE3Quat pose(q,trans);
            g2o::VertexSE3Expmap * v_se3 = new g2o::VertexSE3Expmap();
            if (isFirst){
                v_se3->setFixed(true);
                isFirst = false;
            }
            v_se3->setId(grasphIdCounter);
            kfId2GraphId[frame->id] = grasphIdCounter;
            grasphIdCounter++;
            v_se3->setEstimate(pose);
            mOptimizer.addVertex(v_se3);
        }

        std::cout << "Registered " << mOptimizer.vertices().size() << " vertices. " << std::endl;
        assert(mOptimizer.vertices().size() == mDataframes.size());

        // ADD Points and projections
        for(auto &frame: mDataframes){
            for (size_t i=0; i<frame->wordsReference.size(); ++i) {
                auto word = frame->wordsReference[i];
                if(word->frames.size() >= mBaminAparitions){
                    if(idsUsed.find(word->id) == idsUsed.end()){
                        idsUsed[word->id]  = true;

                        // Add 3d points
                        g2o::VertexSBAPointXYZ * v_p = new g2o::VertexSBAPointXYZ();
                        v_p->setId(grasphIdCounter);
                        wordId2GraphId[word->id] = grasphIdCounter;
                        grasphIdCounter++;

                        v_p->setMarginalized(true);
                        Eigen::Vector3d point3d(word->point[0],
                                                word->point[1],
                                                word->point[2]);
                        v_p->setEstimate(point3d);
                        mOptimizer.addVertex(v_p);

                        // Add projections
                        for (size_t j=0; j<word->frames.size(); ++j){
                            auto projection = word->projections[word->frames[j]];
                            Eigen::Vector2d z(projection[0], projection[1]);

                            g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();

                            auto other_v_p = mOptimizer.vertices().find(kfId2GraphId[word->frames[j]])->second; // Get both sides of edge
                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*> (other_v_p));

                            e->setMeasurement(z);
                            e->information() = Eigen::Matrix2d::Identity();

                            e->setParameterId(0, 0);    // ?????????????????????

                            mOptimizer.addEdge(e);
                        }
                    }
                }
            }     
        }


        mOptimizer.initializeOptimization();
        mOptimizer.save("g2o_graph.g2o");

    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void BundleAdjuster_g2o<PointType_>::keyframes(  typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_begin, 
                                                            typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_end) {
    
    
    }

}

#endif