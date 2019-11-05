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
    inline Dataframe<PointType_>::Dataframe(int _id): id_(_id){

    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::appendCovisibility(int _clusterId){
        if(std::find(covisibility_.begin(), covisibility_.end(), _clusterId) == covisibility_.end()){
                        covisibility_.push_back(_clusterId);
        }
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::updateMMI(int _dfId, int _cfId){
        // multimatchesInliersCfs[_cfId] = multimatchesInliersDfs[_dfId];
        assert(false); //  666 DONT KNOW WHAT THE HELL IS THIS 
    }
    
    template<typename PointType_>
    inline void Dataframe<PointType_>::updatePose(Eigen::Matrix4f &_pose){
            pose          = _pose;
            position      = _pose.block<3,1>(0,3);
            orientation   = Eigen::Quaternionf(_pose.block<3,3>(0,0));
        }

    template<typename PointType_>
    inline void Dataframe<PointType_>::addWord(std::shared_ptr<Word<PointType_>> &_word){
        wordsReference_[_word->id] = _word;
    }
 
    template<typename PointType_>
    inline void Dataframe<PointType_>::eraseWord(std::shared_ptr<Word<PointType_>> &_word){
        wordsReference_.erase(_word->id);
    }


    template<typename PointType_>
    inline void Dataframe<PointType_>::pose(Eigen::Matrix4f &_pose){
        pose_          = _pose;
        position_      = _pose.block<3,1>(0,3);
        orientation_   = Eigen::Quaternionf(_pose.block<3,3>(0,0));
    }

    template<typename PointType_>
    inline Eigen::Matrix4f Dataframe<PointType_>::pose() const{
        return pose_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::cloud(pcl::PointCloud<PointType_>::Ptr &_ could){
        cloud_ = _cloud;
    }

    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr Dataframe<PointType_>::cloud() const{
        return cloud_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::featureCloud(pcl::PointCloud<PointType_>::Ptr &_cloud){
        featureCloud_ = _cloud;
    }

    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr Dataframe<PointType_>::featureCloud() const{
        return featureCloud_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::featureDescriptors(cv::Mat &_descriptors){
        featureDescriptors_ = _descriptors;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::featureDescriptors() const{
        return featureDescriptors_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::featureProjections(std::vector<cv::Point2f> &_projs){
        featureProjections = _projs;
    }

    template<typename PointType_>
    inline std::vector<cv::Point2f> Dataframe<PointType_>::featureProjections() const{
        return featureProjections_;
    }

    template<typename PointType_>
    inline bool Dataframe<PointType_>::isOptimized() const{
        return optimized_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::isOptimized(bool _opt){
        optimized_ = _opt;
    }


    template<typename PointType_>
    inline void Dataframe<PointType_>::leftImage(cv::Mat &_image){
        left_ = _image;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::leftImage() const{
        return left_;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::rightImage() const{
        return right_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::rightImage(cv::Mat &_image){
        right_ = _image;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::depthImage() const{
        return depth_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::depthImage(cv::Mat &_image){
        depth_ = _image;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::intrinsics() const{
        return intrinsics_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::instrinsics(cv::Mat &_intrinsics){
        intrinsics_ = _intrinsics;
    }

    template<typename PointType_>
    inline cv::Mat Dataframe<PointType_>::distCoeff() const{
        return distCoeff_;
    }

    template<typename PointType_>
    inline void Dataframe<PointType_>::distCoeff(cv::Mat &_coeff){
        distCoeff_ = _coeff;
    }


}