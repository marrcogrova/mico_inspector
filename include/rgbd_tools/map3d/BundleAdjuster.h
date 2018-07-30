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

#ifndef RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_
#define RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_

#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/ClusterFrames.h>

namespace rgbd{
    template<typename PointType_>
    class BundleAdjuster{
    public:
        virtual bool optimize()=0;
        virtual bool optimizeClusterframes()=0;
        virtual void keyframes(std::vector<std::shared_ptr<DataFrame<PointType_>>> &_keyframes)=0;
        virtual void keyframes(typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_begin, typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_end)=0;
        virtual void clusterframe(std::shared_ptr<ClusterFrames<PointType_>> &_clusterframe)=0;

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

    protected:
        // Parameters of Bundle Adjustment.
        double      mBaMinError = 1e-10;
        unsigned    mBaIterations = 500;
        unsigned    mBaMinAparitions = 5;
    };
}   // namespace rgbd

#include <rgbd_tools/map3d/BundleAdjuster.inl>

#endif //RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_
