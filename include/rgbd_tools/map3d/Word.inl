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

namespace rgbd
{   

    template<typename PointType_>
    inline Word<PointType_>::Word(int _wordId, std::vector<float> _point3D, cv::Mat _descriptor, int _clusterId, std::shared_ptr<ClusterFrames<PointType_>> _clusterframe){
        id = _wordId;
        point = _point3D;
        descriptor = _descriptor;
        clusters.push_back(_clusterId);
        clustermap[_clusterId] = _clusterframe;
    }

    template<typename PointType_>
    inline void Word<PointType_>::addDataframe(int _frameId, std::vector<float> _projections, int _idx){
        frames.push_back(_frameId);
        projections[_frameId] = _projections;
        projectionsEnabled[_frameId] = true;
        idxInKf[_frameId] = _idx;
    }

    template<typename PointType_>
    inline void Word<PointType_>::addClusterframe(int _clusterId, std::shared_ptr<ClusterFrames<PointType_>> _clusterframe){
        clusters.push_back(_clusterId);
        clustermap[_clusterId] = _clusterframe;
    }


    template <typename PointType_>
    inline bool Word<PointType_>::eraseProjection(int _dfId , int _clusterId){
        if (projections.find(_clusterId) != projections.end())
        {
            projections.erase(_dfId);
            idxInKf.erase(_dfId);
            clusters.erase(std::remove(clusters.begin(), clusters.end(), _clusterId), clusters.end());
            frames.erase(std::remove(frames.begin(), frames.end(), _dfId), frames.end());
            return true;
        }
        else
        {
            return false;
        }
    }
    
    template <typename PointType_>
    inline void Word<PointType_>::updateNormal(){
        Eigen::Vector3f wordPos(point[0],point[1],point[2]);
            Eigen::Vector3f normal;
            int nClust=0;
            for(auto& cluster:clustermap){
                Eigen::Matrix4f clusterPose=cluster.second->bestPose();
                Eigen::Vector3f clusterPosition = clusterPose.block<3,1>(0,3);
                Eigen::Vector3f partialWordNormal = wordPos - clusterPosition;
                normal = normal + partialWordNormal/partialWordNormal.norm();
                nClust++;
            }
            normalVector=normal/nClust;
    }

    template <typename PointType_>
    inline void Word<PointType_>::checkProjections(){
        updateNormal();
        Eigen::Vector3f wordPos(point[0],point[1],point[2]);
        for(auto& cluster:clustermap){
            Eigen::Matrix4f clusterPose=cluster.second->bestPose();
            Eigen::Vector3f clusterPosition = clusterPose.block<3,1>(0,3);
            Eigen::Vector3f partialWordNormal = wordPos - clusterPosition;
            partialWordNormal = partialWordNormal/partialWordNormal.norm();
            Eigen::Vector3f wordNormalMean = normalVector/normalVector.norm();
            if(cos(45*M_PI/180)<abs(partialWordNormal.dot(wordNormalMean))){
                auto df = cluster.second->bestDataframePtr();
                eraseProjection(df->id,cluster->id);
            }
        }
    }
} // namespace rgbd