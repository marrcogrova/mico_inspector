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

#ifndef SLAMMARKI_VISUALIZER_H_
#define SLAMMARKI_VISUALIZER_H_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>

#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/ClusterFrames.h>
#include <rgbd_tools/map3d/Database.h>
#include <rgbd_tools/cjson/json.h>

#include <pcl/octree/octree_pointcloud_occupancy.h>
#include <rgbd_tools/utils/LogManager.h>

namespace rgbd {
    template <typename PointType_>
    class Visualizer
    {
    public:
        /// Initializes visualizer parameters
        static bool init(cjson::Json _configFile, Database<PointType_> *_database);
        
        static Visualizer* get(){
            return mSingleton;
        }

        void cleanAll(){
            if(!mViewer)
                return;
                
            mViewer->removeAllPointClouds();
            mViewer->removeAllCoordinateSystems();
            mViewer->removeAllShapes();
        }


        void drawDataframe(std::shared_ptr<rgbd::DataFrame<PointType_>> &_kf);
        void updateDataframe(int _frameId, Eigen::Matrix4f &_newPose);
        void drawClusterframe(std::shared_ptr<rgbd::ClusterFrames<PointType_>> &_cluster, bool _drawPoints = false);
        void updateClusterframe(int _clusterId, Eigen::Matrix4f &_newPose);
        void drawWords(std::map<int, std::shared_ptr<Word<PointType_>>> _words);

        // Draw every word optimized
        bool draw3DMatches(pcl::PointCloud<PointType_> _pc1, pcl::PointCloud<PointType_> _pc2);
        
        // Draw every word optimized
        bool updateCurrentPose(Eigen::Matrix4f &_pose);

        void pause();
        void spinOnce();
        void saveScreenshot();
        void keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data);
        void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void);
        void pointPickedCallback(const pcl::visualization::PointPickingEvent &event,void*viewer_void);
        boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;

        typedef std::function<void(const pcl::visualization::KeyboardEvent &, void *)> CustomCallbackType;
        void addCustomKeyCallback(CustomCallbackType _callback);

    private:
        void insertNodeCovisibility(Eigen::Vector3f &_position);
        void updateNodeCovisibility(int _id, Eigen::Vector3f &_position);
        void addCovisibility(int _id, std::vector<int> &_others);
    
    private:
        Visualizer(cjson::Json _configFile, Database<PointType_> *_database);

        static Visualizer * mSingleton;

        float x,y,z;
        typename pcl::octree::OctreePointCloudOccupancy<PointType_> mOctreeVis;
        typedef typename pcl::octree::OctreePointCloudOccupancy<PointType_>::Iterator OctreeIterator;
        bool mUseOctree = false;
        int mOctreeDepth = 1;
        std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> mClustersFrames;
        Database<PointType_> *mDatabase;
        typename pcl::PointCloud<PointType_>::Ptr wordCloud = typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>());
        bool mDenseVisualization=false;
        bool mPause = false;

        std::vector<CustomCallbackType> mCustomCallbacks;

        vtkSmartPointer<vtkPolyData> mCovisibilityGraph;
        vtkSmartPointer<vtkUnsignedCharArray> mCovisibilityNodeColors;
        vtkSmartPointer<vtkPoints> mCovisibilityNodes;

        std::map<int,bool> mExistingCluster;
        std::map<int,int> mNodeCovisibilityCheckSum;
        bool mTrackCamera = false;
        bool mUseVoxel = false;
        pcl::VoxelGrid<PointType_> mVoxeler;
        
    };
} // namespace rgbd

#include "Visualizer.inl"

#endif // SLAMMARKI_VISUALIZER_H_