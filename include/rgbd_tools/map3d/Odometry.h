//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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

#ifndef RGBDTOOLS_MAP3D_ODOMETRY_H_
#define RGBDTOOLS_MAP3D_ODOMETRY_H_

#include <rgbd_tools/cjson/json.h>
#include <rgbd_tools/map3d/DataFrame.h>

#include <rgbd_tools/utils/LogManager.h>

namespace rgbd{
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Cout>
    class Odometry : public LoggableInterface<DebugLevel_, OutInterface_> {
    public:
        /// Initializes parameters
        virtual bool init(cjson::Json _configFile) = 0;

        /// Pick up an image from the camera and get a keyframe with the point cloud and feature cloud
        virtual bool computeOdometry(std::shared_ptr<rgbd::DataFrame<PointType_>> _prevDf, std::shared_ptr<rgbd::DataFrame<PointType_>> _currentDf) = 0;

        /// Pick up an image from the camera and get a keyframe with the point cloud and feature cloud
        virtual bool computeOdometry(std::shared_ptr<rgbd::ClusterFrames<PointType_>> _prevCf, std::shared_ptr<rgbd::DataFrame<PointType_>> _currentCf) = 0;

    };
}

#endif