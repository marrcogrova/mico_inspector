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


#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>

#include <Eigen/Eigen>

namespace rgbd{
    template<typename PointType_>
    class BundleAdjuster_g2o{
    public:
        BundleAdjuster_g2o();

        bool optimize();
        void keyframes(std::vector<std::shared_ptr<DataFrame<PointType_>>> &_keyframes); // FUTURE IMPLEMENTATION WILL KEEP TRACK OF THE GRAPH !!
        void keyframes(typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_begin, typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_end);

        // ---- Getters ----
        /// \brief Get minimum error set as stopping criteria for the Bundle Adjustment process.
        /// \return minimum error.
        double      minError       () const;

        /// \brief Get number of iterations set as stopping criteria for the Bundle Adjustment process.
        /// \return iterations.
        unsigned    iterations     () const;

        /// \brief Get minumim number of times that a points needs to be observed to be used in the Bundle Adjustment.
        /// \return number of aparitions.
        unsigned    minAparitions  () const;

        // ---- Setters ----
        /// \brief Set minimum error set as stopping criteria for the Bundle Adjustment process.
        /// \param _error: minimum error.
        void minError         (double _error);

        /// \brief Set number of iterations set as stopping criteria for the Bundle Adjustment process.
        /// \param _iterations iterations.
        void iterations       (unsigned _iterations);

        /// \brief Set minumim number of times that a points needs to be observed to be used in the Bundle Adjustment.
        /// \param _aparitions: number of aparitions.
        void minAparitions    (unsigned _aparitions);
    private:


    private:
        // Parameters of Bundle Adjustment.
        double      mBaMinError = 1e-10;
        unsigned    mBaIterations = 10;
        unsigned    mBaminAparitions = 3;

        std::vector<std::shared_ptr<DataFrame<PointType_>>> mDataframes;

        g2o::SparseOptimizer mOptimizer;
        g2o::OptimizationAlgorithmLevenberg *mSolverPtr; 
    };
}

#include <rgbd_tools/map3d/BundleAdjuster_g2o.inl>

#endif
