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


namespace rgbd{
    template<typename PointType_>
    inline ClusterFrames<PointType_>::ClusterFrames(std::shared_ptr<DataFrame<PointType_>> &_df, int _id){
        id = _id;
        frames.push_back(_df->id);

        // Add df feature descriptors to cluster
        featureDescriptors = _df->featureDescriptors.clone();
        
        intrinsic = _df->intrinsic;
        distCoeff = _df->coefficients;
        
        dataframes[_df->id] = _df;
        bestDataframe = _df->id;
    }

    template<typename PointType_>
    inline void ClusterFrames<PointType_>::addDataframe(std::shared_ptr<DataFrame<PointType_>> &_df){
        frames.push_back(_df->id);

        // Append df feature projetions to cluster
        featureProjections.insert(featureProjections.end(), _df->featureProjections.begin(), _df->featureProjections.end());
        // Add df feature descriptors to cluster
        featureDescriptors.push_back(_df->featureDescriptors);

        dataframes[_df->id] = _df;
    }
}