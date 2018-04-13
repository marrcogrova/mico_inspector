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


#ifndef RGBDTOOLS_MAP3D_DATABASE_H_
#define RGBDTOOLS_MAP3D_DATABASE_H_

#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/ClusterFrames.h>
#include <rgbd_tools/map3d/Word.h>
#include <unordered_map>
#include <map>
#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif

namespace rgbd{

    template<typename PointType_>
    class Database{
    public:
        /// Add dataframe to database
        bool addDataframe(std::shared_ptr<DataFrame<PointType_>> &_kf,double _mk_nearest_neighbors,double _mRansacMaxDistance,int _mRansacIterations,int _mRansacMinInliers,double _mFactorDescriptorDistance);
        //void connectKeyframes(unsigned _id1, unsigned _id2, bool _has3D = true);

        /// Change relationship between clusters
        void changeRelations(int id, int mate, double affinity);

        /// Reset database. Erase dataframes and clusterframes
        void reset();

        /// \brief init vocabulary
        /// \param _path: path to file containing the vocabulary
        /// \return true if initialized, false if not.
        bool initVocabulary(const std::string &_path);

        /// Create and update cluster words
        void wordCreation();

        /// Get list with all cluster of frames
        std::vector<std::shared_ptr<DataFrame<PointType_>>>  dataframes       ()              {return mDataframes; }

        /// Get specific cluster from id
        /// \param _id: id of desired frame
        std::shared_ptr<DataFrame<PointType_>>               dataframe        (unsigned _id)  {return mDataframes[_id];}

        /// Get list with all dataframes
        std::unordered_map<int, std::shared_ptr<ClusterFrames<PointType_>>>  clusters       ()              {return mClustersMap; }

        /// Get pointer to last cluster created
        std::shared_ptr<ClusterFrames<PointType_>>           rlastCluster() const {return mLastCluster;}

        /// Get specific frame from id
        /// \param _id: id of desired frame
        std::shared_ptr<ClusterFrames<PointType_>>               cluster        (unsigned _id)  {return mClustersMap[_id];}

        /// Get number of dataframe registered
        unsigned                                            numKeyframes    ()              {return mDataframes.size();}

        /// Get number of clusters registereds
        unsigned                                            numClusters     ()              {return mClustersMap.size();}

        /// Get dictionary of words
        std::unordered_map<int, std::shared_ptr<Word>> dictionary(){return mWordDictionary;}

        /// Get cloud of words
        pcl::PointCloud<PointType_> wordMap(){return mWordMap;}

        /// Reset last cluster pointer
        void resetCluster(){
         mLastCluster = nullptr;
        }

        ///Change DBoW2 score for testing purposes
        void changeDBow2Score(int newScore){dbow2Score=newScore;}


    private:
        int dbow2Score=-1;
        std::unordered_map<int, std::shared_ptr<ClusterFrames<PointType_>>> mClustersMap;
        std::shared_ptr<ClusterFrames<PointType_>>              mLastCluster;
        std::vector<std::shared_ptr<DataFrame<PointType_>>>     mDataframes;
        std::unordered_map<int, std::shared_ptr<Word>>          mWordDictionary;
        std::shared_ptr<Word>                                   mLastWord;
        pcl::PointCloud<PointType_>                             mWordMap;

        #ifdef USE_DBOW2
            OrbVocabulary mVocabulary;
        #endif

    private: // helper functions
        Eigen::Vector3f triangulateFromProjections(std::unordered_map<int, std::vector<float>>);
    };
}

#include "Database.inl"

#endif
