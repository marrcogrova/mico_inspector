//---------------------------------------------------------------------------------------------------------------------
//  mico
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


namespace mico {
    template<typename PointType_>
    inline ClusterFrames<PointType_>::ClusterFrames(std::shared_ptr<Dataframe<PointType_>> &_df, int _id){
        id = _id;
        frames.push_back(_df->id);

        // Add df feature descriptors to cluster
        featureDescriptors = _df->featureDescriptors.clone();
        featureProjections.insert(featureProjections.end(), _df->featureProjections.begin(), _df->featureProjections.end());
        
        updatePose(_df->pose);

        left = _df->left.clone();
        cloud = _df->cloud;
        featureCloud = _df->featureCloud;

        intrinsic = _df->intrinsic;
        distCoeff = _df->coefficients;
        
        multimatchesInliersCfs = _df->multimatchesInliersCfs;

        dataframes[_df->id] = _df;
    }

    template<typename PointType_>
    inline void ClusterFrames<PointType_>::addDataframe(std::shared_ptr<Dataframe<PointType_>> &_df){
        frames.push_back(_df->id);
    }

    template<typename PointType_>
    inline void ClusterFrames<PointType_>::updateCovisibility(int _clusterId){
        if(std::find(covisibility.begin(), covisibility.end(), _clusterId) == covisibility.end()){
                        covisibility.push_back(_clusterId);
        }
    }

    template<typename PointType_>
    inline void ClusterFrames<PointType_>::updateMMI(int _dfId, int _cfId){
        multimatchesInliersCfs[_cfId] = multimatchesInliersDfs[_dfId];
    }
    
    template<typename PointType_>
    inline void ClusterFrames<PointType_>::updatePose(Eigen::Matrix4f &_pose){
            pose          = _pose;
            position      = _pose.block<3,1>(0,3);
            orientation   = Eigen::Quaternionf(_pose.block<3,3>(0,0));
        }

    template<typename PointType_>
    inline void ClusterFrames<PointType_>::addWord(std::shared_ptr<Word<PointType_>> &_word){
        wordsReference[_word->id] = _word;
    }
 
    template<typename PointType_>
    inline void ClusterFrames<PointType_>::eraseWord(std::shared_ptr<Word<PointType_>> &_word){
        wordsReference.erase(_word->id);
    }

    template<typename PointType_>
    inline Eigen::Matrix4f ClusterFrames<PointType_>::getPose(){
        return pose;
    }

    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr  ClusterFrames<PointType_>::getCloud(){
        return cloud;
    }
 
    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr ClusterFrames<PointType_>::getFeatureCloud(){
        return featureCloud;
    }

}