#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <mico/base/map3d/utils3d.h>
#include <pcl/io/pcd_io.h>

namespace mico {

    template <typename PointType_>
    inline bool SceneVisualizer<PointType_>::init(cjson::Json _configFile)//, DatabaseCF<PointType_> *_database)
    {
        if(_configFile.contains("enable") && (bool) _configFile["enable"]){
            // mDatabase  = _database;
            mViewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
            mViewer->setBackgroundColor(0.1, 0.1, 0.1, 0);
            mViewer->addCoordinateSystem(0.05, "base", 0);
            mViewer->addCoordinateSystem(0.02, "current_pose", 0);
            mViewer->registerKeyboardCallback(&SceneVisualizer::keycallback, *this, (void *)&mViewer);
            mViewer->registerMouseCallback(&SceneVisualizer::mouseEventOccurred, *this, (void *)&mViewer);
            mViewer->registerPointPickingCallback(&SceneVisualizer::pointPickedCallback, *this, (void*)&mViewer);
            mViewer->setCameraPosition (1.59696, 0.285761, -3.40482, -0.084178, -0.989503, -0.117468);
            // if (mDatabase == nullptr) {
            //     //std::cout << "[SceneVisualizer] Failing getting database instance" << std::endl;
            // }
            if (_configFile.contains("DenseVisualization")) {
                mDenseVisualization = (bool) _configFile["DenseVisualization"];
            }
            if (_configFile.contains("use_octree")) {
                mUseOctree = (bool) _configFile["use_octree"];
                mOctreeDepth = _configFile["octree_depth"];
            }else{
            }

            if(_configFile.contains("voxel_size")){
                mUseVoxel = true;
                double mVoxelSize = (double) _configFile["voxel_size"];
                mVoxeler.setLeafSize (mVoxelSize,mVoxelSize,mVoxelSize);

            }

            mCovisibilityNodeColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            mCovisibilityNodeColors->SetNumberOfComponents(3);
            mCovisibilityNodeColors->SetName("Colors");
            mCovisibilityNodes = vtkSmartPointer<vtkPoints>::New();
            mCovisibilityGraph = vtkSmartPointer<vtkPolyData>::New();
            mCovisibilityGraph->Allocate();
            mViewer->addModelFromPolyData(mCovisibilityGraph, "covisibility_graph");
        }
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::drawDataframe(std::shared_ptr<mico::DataFrame<PointType_>> &_df){
        if(!mViewer)
            return;

        if (mViewer->contains("dataframe_cs_" + std::to_string(_df->id))) {
            mViewer->removeCoordinateSystem("dataframe_cs_" + std::to_string(_df->id));
            mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(_df->pose), "dataframe_cs_" + std::to_string(_df->id));
            mViewer->updatePointCloudPose("dataframe_cloud_" + std::to_string(_df->id), Eigen::Affine3f(_df->pose));
            
            mViewer->removeText3D("dataframe_text_" + std::to_string(_df->id));
            pcl::PointXYZ position(_df->pose(0, 3), _df->pose(1, 3), _df->pose(2, 3));
            mViewer->addText3D(std::to_string(_df->id), position, 0.01, 0.8, 0.2, 0.8, "dataframe_text_" + std::to_string(_df->id));

            // mViewer->updatePointCloudPose("dataframe_words_" + std::to_string(_df->id), Eigen::Affine3f(_df->pose)); 666 Hummmm already in global coordinates
        }
        else {
            mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(_df->pose), "dataframe_cs_" + std::to_string(_df->id));

            // Draw feature cloud
            if (_df->cloud != nullptr) {
                mViewer->addPointCloud<PointType_>(_df->cloud, "dataframe_cloud_" + std::to_string(_df->id));
                mViewer->updatePointCloudPose("dataframe_cloud_" + std::to_string(_df->id), Eigen::Affine3f(_df->pose));

                pcl::PointXYZ position(_df->pose(0, 3), _df->pose(1, 3), _df->pose(2, 3));
                mViewer->addText3D(std::to_string(_df->id), position, 0.01, 0.8, 0.2, 0.8, "dataframe_text_" + std::to_string(_df->id));

                if (!_df->wordsReference.empty()) {
                    typename pcl::PointCloud<PointType_>::Ptr cloudDictionary = typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
                    for (auto &w : _df->wordsReference) {
                        std::shared_ptr<Word<PointType_>> word = w;
                        cloudDictionary->push_back(word->asPclPoint());
                    }
                    // Draw dictionary cloud
                    mViewer->addPointCloud<PointType_>(cloudDictionary, "dataframe_words_" + std::to_string(_df->id));
                    mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "dataframe_words_" + std::to_string(_df->id));
                    mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "dataframe_words_" + std::to_string(_df->id));
                }
            }
        }
        mViewer->spinOnce(10, true);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::updateDataframe(int _frameId, Eigen::Matrix4f &_newPose){
        if(!mViewer)
            return;

        if (mViewer->contains("dataframe_cs_" + std::to_string(_frameId))) {
            mViewer->removeCoordinateSystem("dataframe_cs_" + std::to_string(_frameId));
            mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(_newPose), "dataframe_cs_" + std::to_string(_frameId));
            mViewer->updatePointCloudPose("dataframe_cloud_" + std::to_string(_frameId), Eigen::Affine3f(_newPose));
            mViewer->removeText3D("dataframe_text_" + std::to_string(_frameId));
            pcl::PointXYZ position(_newPose(0, 3), _newPose(1, 3), _newPose(2, 3));
            mViewer->addText3D(std::to_string(_frameId), position, 0.01, 0.8, 0.2, 0.8, "dataframe_text_" + std::to_string(_frameId));
            // mViewer->updatePointCloudPose("dataframe_words_" + std::to_string(_frameId), Eigen::Affine3f(_newPose)); 666 Hummmm already in global coordinates
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::drawClusterframe(std::shared_ptr<mico::ClusterFrames<PointType_>> &_cluster, bool _drawPoints){
        if(!mViewer){
            return;
        }

        if(mExistingCluster.find(_cluster->id) != mExistingCluster.end()){
            mViewer->removeCoordinateSystem("clusterframe_cs_" + std::to_string(_cluster->id));
            mViewer->removeText3D("clusterframe_text_" + std::to_string(_cluster->id));
            if(mUseOctree)
                mViewer->removePointCloud("octree");
            else
                mViewer->removePointCloud("clusterframe_cloud_" + std::to_string(_cluster->id));
        }

        Eigen::Matrix4f clusterPose = _cluster->getPose();
        mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(clusterPose), "clusterframe_cs_" + std::to_string(_cluster->id));
        
        pcl::PointXYZ position(clusterPose(0, 3), clusterPose(1, 3), clusterPose(2, 3));
        mViewer->addText3D(std::to_string(_cluster->id), position, 0.015, 1,0,0, "clusterframe_text_" + std::to_string(_cluster->id));
        
        if(_drawPoints){
            mViewer->removePointCloud("clusterframe_words_" + std::to_string(_cluster->id));
            if (!_cluster->wordsReference.empty()) {
                typename pcl::PointCloud<PointType_>::Ptr cloudDictionary = typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
                for (auto &w : _cluster->wordsReference) {
                    std::shared_ptr<Word<PointType_>> word = w.second;
                    cloudDictionary->push_back(word->asPclPoint());
                }
                // Draw dictionary cloud
                mViewer->addPointCloud<PointType_>(cloudDictionary, "clusterframe_words_" + std::to_string(_cluster->id));  
                mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "clusterframe_words_" + std::to_string(_cluster->id));
                mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "clusterframe_words_" + std::to_string(_cluster->id));
            }
        }
        

        // Draw cloud
        if (_cluster->getCloud() != nullptr){ 
            if(mUseOctree){
                pcl::PointCloud<PointType_> cloud;
                pcl::transformPointCloudWithNormals(*_cluster->getCloud(), cloud, clusterPose);
                mOctreeVis.setInputCloud (cloud.makeShared());
                mOctreeVis.addPointsFromInputCloud ();

                OctreeIterator treeIt;
                OctreeIterator treeItEnd = mOctreeVis.end();


                
                // int depth = mOctreeVis.getTreeDepth() / 1.25;

                pcl::PointCloud<PointType_> denseCloud;
                PointType_ pt;
                Eigen::Vector3f voxel_min, voxel_max;
                for (treeIt = mOctreeVis.begin(mOctreeDepth); treeIt!=treeItEnd; ++treeIt) {
                    mOctreeVis.getVoxelBounds(treeIt, voxel_min, voxel_max);

                    pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
                    pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
                    pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
                    
                    denseCloud.push_back(pt);
                }

                mViewer->addPointCloud<PointType_>(denseCloud.makeShared(), "octree");
                mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "octree");
            }
            else{
                pcl::PointCloud<PointType_> cloudDrawn;
                if(mUseVoxel){
                    mVoxeler.setInputCloud (_cluster->getCloud());
                    mVoxeler.filter (cloudDrawn);
                    mViewer->addPointCloud<PointType_>(cloudDrawn.makeShared(), "clusterframe_cloud_" + std::to_string(_cluster->id));
                }else{
                    mViewer->addPointCloud<PointType_>(_cluster->getCloud(), "clusterframe_cloud_" + std::to_string(_cluster->id));
                }
                mViewer->updatePointCloudPose("clusterframe_cloud_" + std::to_string(_cluster->id), Eigen::Affine3f(clusterPose));
            }
        }

        // Draw covisibility.
        // std::cout << "Drawing covisibility. Existing prevous "<< mExistingCluster.size() << " nodes." << std::endl;
        Eigen::Vector3f origin = {position.x, position.y, position.z};
        if(mExistingCluster.find(_cluster->id) != mExistingCluster.end()){
            // std::cout << "Updating existing covisibility" << std::endl;
            updateNodeCovisibility(_cluster->id, origin);
            if(_cluster->covisibility.size() != mNodeCovisibilityCheckSum[_cluster->id]){
                std::vector<int> newCov(_cluster->covisibility.begin()+ mNodeCovisibilityCheckSum[_cluster->id], 
                                        _cluster->covisibility.end());
                addCovisibility(_cluster->id, newCov);
                mNodeCovisibilityCheckSum[_cluster->id] = _cluster->covisibility.size();
            }
        }else{
            // std::cout << "Created new node" << std::endl;
            insertNodeCovisibility(origin);
            addCovisibility(_cluster->id, _cluster->covisibility);
            mNodeCovisibilityCheckSum[_cluster->id] = _cluster->covisibility.size();
            
            mCovisibilityGraph->SetPoints(mCovisibilityNodes);
            mCovisibilityGraph->GetPointData()->SetScalars(mCovisibilityNodeColors);
        }
        mExistingCluster[_cluster->id] = true;

        mCovisibilityNodes->Modified();

        mViewer->spinOnce(10, true);
    }


    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::drawWords(std::map<int, std::shared_ptr<Word<PointType_>>> _words){
        mViewer->removePointCloud("words");
        if (!_words.empty()) {
            typename pcl::PointCloud<PointType_>::Ptr cloudDictionary = typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
            for (auto &w : _words) {
                std::shared_ptr<Word<PointType_>> word = w.second;
                cloudDictionary->push_back(word->asPclPoint());
            }
            // Draw dictionary cloud
            mViewer->addPointCloud<PointType_>(cloudDictionary, "words");  
            mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "words");
            mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "words");
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::updateClusterframe(int _clusterId, Eigen::Matrix4f &_newPose){
        if(!mViewer)
            return;
            
        if (mViewer->contains("clusterframe_cs_" + std::to_string(_clusterId))) {
            mViewer->removeCoordinateSystem("clusterframe_cs_" + std::to_string(_clusterId));
            mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(_newPose), "clusterframe_cs_" + std::to_string(_clusterId));

            mViewer->removeText3D("clusterframe_text_" + std::to_string(_clusterId));
            pcl::PointXYZ position(_newPose(0, 3), _newPose(1, 3), _newPose(2, 3));
            mViewer->addText3D(std::to_string(_clusterId), position, 0.015, 1,0,0, "text_" + std::to_string(_clusterId));

            mViewer->updatePointCloudPose("clusterframe_cloud_" + std::to_string(_clusterId), Eigen::Affine3f(_newPose));
            //mViewer->updatePointCloudPose("clusterframe_words_" + std::to_string(_clusterId), Eigen::Affine3f(_newPose));  666 Hummmm already in global coordinates
        }
    }


    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline bool SceneVisualizer<PointType_>::draw3DMatches(pcl::PointCloud<PointType_> _pc1, pcl::PointCloud<PointType_> _pc2){
        if(!mViewer)
            return false;
            
        assert(_pc1.size() == _pc2.size());

        for(unsigned i = 0 ; i<_pc1.size(); i++){
            mViewer->addLine(_pc1[i], _pc2[i], "line_"+std::to_string(i));
        }
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline bool SceneVisualizer<PointType_>::updateCurrentPose(Eigen::Matrix4f &_pose){
        if(!mViewer)
            return false;
            
        mViewer->removeCoordinateSystem("current_pose");
        mViewer->addCoordinateSystem(0.02, Eigen::Affine3f(_pose), "current_pose");
        mViewer->removeText3D("current_pose_text");
        pcl::PointXYZ position(_pose(0, 3), _pose(1, 3), _pose(2, 3));
        mViewer->addText3D("current_pose", position, 0.01, 1.0, 0.2, 0.2, "current_pose_text");
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::insertNodeCovisibility(Eigen::Vector3f &_position){
        const unsigned char green[3] = {0, 255, 0};

        mCovisibilityNodes->InsertNextPoint(    _position[0], 
                                                _position[1], 
                                                _position[2]);
        mCovisibilityNodeColors->InsertNextTupleValue(green);
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::updateNodeCovisibility(int _id, Eigen::Vector3f &_position){
        double point[3] = {_position[0],_position[1], _position[2]};
        mCovisibilityNodes->SetPoint(_id, point);
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::addCovisibility(int _id, std::vector<int> &_others){
        for(auto &other:_others){ 
            vtkIdType connectivity[2];
            connectivity[0] = _id;
            connectivity[1] = other;
            mCovisibilityGraph->InsertNextCell(VTK_LINE,2,connectivity); //Connects the first and fourth point we inserted into a line
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::pause() {
        if(!mViewer)
            return;
            
        mPause=true;
        while (mPause) {
            mViewer->spinOnce(10, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            cv::waitKey(10);
        }
    };

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::spinOnce(){
        if(!mViewer)
            return;
            
        mViewer->spinOnce(10, true);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data) {
        for(auto &callback:mCustomCallbacks){
            callback(_event, _data);
        }

        if (_event.keyDown() && _event.getKeySym() == "z") {
            std::cout << "[Visualizer] Toogle pause" << std::endl;
            if(mPause==true) mPause = false;
            else pause();
        }
        else if (_event.keyDown() && _event.getKeySym() == "v") {
        // #ifdef USE_DBOW2
        //     // Init DBoW2 vocabulary
        //     const int K = 6; // From article Bags of Binary Words for Fast Place Recognition inImage Sequences
        //     const int L = 4;
        //     //const DBoW2::WeightingType weight = DBoW2::TF_IDF;  //TODO: Scoring type and weight type? 
        //     //const DBoW2::ScoringType score = DBoW2::L2_NORM;
        //     //OrbVocabulary voc(K, L, weight, score);
        //     OrbVocabulary voc(K, L);

        //     auto clusters = mDatabase->clusterframes_;
        //     std::vector<std::vector<cv::Mat>> allFeatures;
        //     for (unsigned i = 0; i < clusters.size(); i++) {
        //         allFeatures.push_back(std::vector<cv::Mat>());
        //         for (int r = 0; r < clusters[i]->featureDescriptors.rows; r++) {
        //             allFeatures[i].push_back(clusters[i]->featureDescriptors.row(r));
        //         }
        //     }
        //     voc.create(allFeatures);
        //     voc.save("vocabulary_dbow2_livingroom_orb_k" + std::to_string(K) + "L" + std::to_string(L) + ".xml");
        //     std::cout << "Vocabulary saved in vocabulary_dbow2_livingroom_orb_k" + std::to_string(K) + "L" + std::to_string(L) + ".xml" << std::endl;
        // #else
        //     std::cout << "DBOW 2 not installed, or library not compiled with it. Cant compute dictionarty." << std::endl;
        // #endif
        }
    };

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void){
      //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

      if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
          event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease){}
        
        
    };

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::pointPickedCallback(const pcl::visualization::PointPickingEvent &event,void*viewer_void) {
        // event.getPoint(x,y,z);
        // for(auto &w: mDatabase->wordDictionary_){
        //     if(w.second->point[0]==x && w.second->point[1]==y && w.second->point[2]==z){
        //         std::cout << " Clicked word: " + std::to_string(w.second->id) << std::endl;
        //     }
        // }
        // std::cout << "Point clicked at position (" << x << ", " << y << ", " << z << ")" << std::endl;  
    }; 

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_>
    inline void SceneVisualizer<PointType_>::addCustomKeyCallback(CustomCallbackType _callback){
        mCustomCallbacks.push_back(_callback);
    }
} // namespace mico