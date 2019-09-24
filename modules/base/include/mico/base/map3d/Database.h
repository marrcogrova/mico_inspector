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

#ifndef MICO_BASE_MAP3D_DATABASE_H_
#define MICO_BASE_MAP3D_DATABASE_H_

#include <mico/base/map3d/DataFrame.h>
#include <mico/base/map3d/ClusterFrames.h>
#include <mico/base/utils/LogManager.h>
#include <mico/base/cjson/json.h>


namespace mico {
    template <typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Cout>
    class Database : public LoggableInterface<DebugLevel_, OutInterface_>{
    public:
        /// \brief init vocabulary
        /// \param _path: path to file containing the vocabulary
        /// \return true if initialized, false if not.
        bool init(const cjson::Json &_configFile);

        /// Create new ClusterFrame
        bool createCluster(std::shared_ptr<mico::DataFrame<PointType_>> _df);

        /// Score df to cluster
        double dfToClusterScore(std::shared_ptr<mico::DataFrame<PointType_>> _df);

        /// Write signature of a cluster from his words
        void writeClusterSignature(std::shared_ptr<mico::ClusterFrames<PointType_>> &_cluster);

        /// Save camera pose
        void savePoses(std::string _posesFileName);

        /// Find and create words comparing current dataframe with last dataframe added
        void wordCreation(std::shared_ptr<mico::DataFrame<PointType_>> _df);

        /// Find and create words comparing two clusters
        void wordComparison(std::shared_ptr<mico::ClusterFrames<PointType_>> _queryCluster,
                            std::shared_ptr<mico::ClusterFrames<PointType_>> _trainCluster);

        /// Check duplicated words beetween near clusters
        void clusterComparison(std::map<int,std::shared_ptr<ClusterFrames<PointType_>>> _clusterSubset, bool _localComparison);

        /// Set number of clusters compared  
        void clusterComparison(int _nCluster);

        /// Compute two keyframes to get his transform and matches
        bool addDataframe(std::shared_ptr<mico::DataFrame<PointType_>> _df);

        /// Set score to create clusters
        void clusterScore(double _score);

        /// Return word dictionary
        std::unordered_map<int, std::shared_ptr<Word<PointType_>>> getDictionary();

        // Get map of clusters
        std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> clusterFrames();

        std::shared_ptr<mico::DataFrame<PointType_>> mLastDataFrame=nullptr;
        std::shared_ptr<mico::ClusterFrames<PointType_>> mLastClusterframe=nullptr;
        std::shared_ptr<Word<PointType_>> mLastWord=nullptr;
        std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> mClusterframes;
        std::map<int, std::shared_ptr<Word<PointType_>>> mWordDictionary;
        
        double mScore=0.4;
        int mNumCluster = 0; 
        #ifdef USE_DBOW2
            OrbVocabulary mVocabulary;
        #endif
    };

} // namespace mico 

#include "Database.inl"

#endif // 