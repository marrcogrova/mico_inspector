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

#ifndef RGBDTOOLS_MAP3D_BUNDLEADJUSTERG2O_H_
#define RGBDTOOLS_MAP3D_BUNDLEADJUSTERG2O_H_
#define USE_G2O 1
#ifdef USE_G2O
    #include <g2o/core/sparse_optimizer.h>
    #include <g2o/core/block_solver.h>
    #include <g2o/core/solver.h>
    #include <g2o/core/robust_kernel_impl.h>
    #include <g2o/core/optimization_algorithm_levenberg.h>
    #include <g2o/solvers/cholmod/linear_solver_cholmod.h>
    #include <g2o/solvers/dense/linear_solver_dense.h>
    #include <g2o/types/sba/types_six_dof_expmap.h>
    #include <g2o/solvers/structure_only/structure_only_solver.h>
#endif

#include <rgbd_tools/map3d/BundleAdjuster.h>
#include <Eigen/Eigen>
#include <rgbd_tools/map3d/ClusterFrames.h>


namespace rgbd{
    template<typename PointType_>
    class BundleAdjuster_g2o: public rgbd::BundleAdjuster<PointType_>{
    public:
        BundleAdjuster_g2o();

        bool optimize();
        bool optimizeClusterframes();
        void keyframes(std::vector<std::shared_ptr<DataFrame<PointType_>>> &_keyframes); // FUTURE IMPLEMENTATION WILL KEEP TRACK OF THE GRAPH !!
        void keyframes(typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_begin, typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_end);
        void clusterframe(std::shared_ptr<ClusterFrames<PointType_>> &_clusterframe);
    private:

        std::vector<std::shared_ptr<DataFrame<PointType_>>> mDataframes;
        std::shared_ptr<ClusterFrames<PointType_>> mClusterframe= nullptr;
    #ifdef USE_G2O
        g2o::SparseOptimizer mOptimizer;
        g2o::OptimizationAlgorithmLevenberg *mSolverPtr; 
    #endif
        std::map<int, int> kfId2GraphId;
        std::map<int, int> clusterId2GraphId;
        std::map<int, int> wordId2GraphId;
    };
}

#include <rgbd_tools/map3d/BundleAdjuster_g2o.inl>

#endif
