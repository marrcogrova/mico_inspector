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

#include <mico/base/map3d/utils3d.h>
#include <mico/base/utils/LogManager.h>

#include <opencv2/core/eigen.hpp>

#include <fstream>
#include <iostream>
#include <memory>


namespace mico {

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::init(const cjson::Json &_configFile) {
        mScore = ((double)_configFile["similarity_score"]);
        
        if(_configFile.contains("clusterComparison")) 
            this->clusterComparison((int)_configFile["clusterComparison"]);
            
        #ifdef USE_DBOW2
            if(_configFile.contains("vocabulary")) 
                mVocabulary.load(_configFile["vocabulary"]);
            //mVocabulary.setScoringType(DBoW2::L2_NORM);  //TODO: Change this scoring type
            return !mVocabulary.empty();
        #else
            return false;
        #endif
    }


    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::createCluster(std::shared_ptr<mico::DataFrame<PointType_>> _df) {
        int id = mClusterframes.size();
        // if (mLastClusterframe != nullptr)
        //     id = mLastClusterframe->id + 1;

        std::shared_ptr<ClusterFrames<PointType_>> cluster = std::shared_ptr<ClusterFrames<PointType_>>(new ClusterFrames<PointType_>(_df, id));

        // Update cluster
        mClusterframes[cluster->id] = cluster;

        // Update MMI of previous cluster
        if (id > 1){
            mClusterframes[cluster->id - 1]->updateMMI(_df->id, cluster->id);
            if(mNumCluster>1){
                int n = 0;
                // local cluster comparison
                std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> localClusterSubset;
                localClusterSubset[mLastClusterframe->id] = mLastClusterframe;
                for (auto trainCluster = mClusterframes.rbegin(); trainCluster != mClusterframes.rend() && n <= mNumCluster +2; trainCluster++, n++)
                {   
                    localClusterSubset[trainCluster->first] = trainCluster->second;
                }
                clusterComparison(localClusterSubset,true);
            }
        }
        mLastClusterframe = cluster;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline double DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::dfToClusterScore(std::shared_ptr<mico::DataFrame<PointType_>> _df) {
        #ifdef USE_DBOW2
            // Creating df signature and featVec with DBoW2
            std::vector<cv::Mat> descriptors;
            for (int r = 0; r < _df->featureDescriptors.rows; r++) {
                descriptors.push_back(_df->featureDescriptors.row(r));
            }
            mVocabulary.transform(descriptors, _df->signature, _df->featVec, 4);
            if (mLastClusterframe->signature.empty()) {
                mLastClusterframe->signature = _df->signature;
            }
            // Adding df in current cluster or create a new one
            double score = mVocabulary.score(_df->signature, mLastClusterframe->signature);
            return score;
        #else
            return 0;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::writeClusterSignature(std::shared_ptr<mico::ClusterFrames<PointType_>> &_cluster) {
        std::vector<cv::Mat> descriptors;
        for (auto &w : _cluster->wordsReference) {
            descriptors.push_back(w.second->descriptor);
        }
        mVocabulary.transform(descriptors, _cluster->signature, _cluster->featVec, 4);
    }

    //---------------------------------------------------------------------------------------------------------------------

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::savePoses(std::string _posesFileName)
    {
        // // Open file
        // std::ofstream file(_posesFileName);
        // if (file.is_open())
        // {
        //     // Writting headers
        //     for (auto &cluster : mClustersFrames)
        //     {
        //         Eigen::Matrix4f &cfPose = cluster.second->pose;
        //         Eigen::Quaternionf quat(cfPose.block<3, 3>(0, 0));
        //         file << std::setprecision(4) << " " << cluster.second->pose(0, 3) << " " << cluster.second->pose(1, 3) << " " << cluster.second->pose(2, 3) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
        //     }
        // }
        // else
        // {
        //     std::cout << "[Mapper]: Can't save poses in " << _posesFileName << std::endl;
        // }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::wordCreation(std::shared_ptr<mico::DataFrame<PointType_>> _df) {
        bool isNewCluster = (mLastClusterframe->wordsReference.size() == 0) && (mLastClusterframe->id != 0);
        std::shared_ptr<ClusterFrames<PointType_>> lastCf;
        if (isNewCluster){
            // 666 This only works if clusters are sequential, it might be good if there is a way to identify previous cluster
            lastCf = mClusterframes[mLastClusterframe->id - 1];
        }
        else {
            lastCf = mLastClusterframe;
        }
        auto currentCluster = mLastClusterframe;

        typename pcl::PointCloud<PointType_>::Ptr transformedFeatureCloud(new pcl::PointCloud<PointType_>());
        pcl::transformPointCloud(*lastCf->featureCloud, *transformedFeatureCloud, lastCf->pose);
        std::vector<cv::DMatch> cvInliers = _df->multimatchesInliersCfs[lastCf->id];

        for (unsigned inlierIdx = 0; inlierIdx < cvInliers.size(); inlierIdx++){
            // 666 Assumes that is only matched with previous cloud, loops are not handled in this method
            std::shared_ptr<Word<PointType_>> prevWord = nullptr;
            int inlierIdxInDataframe = cvInliers[inlierIdx].queryIdx;
            int inlierIdxInCluster = cvInliers[inlierIdx].trainIdx;

            // Check if exists a word with the id of the descriptor inlier
            for (auto &w : /*mWordDictionary*/ lastCf->wordsReference){ // TODO: Can we check if the word have current cluster id as key?
                if (w.second->idxInCf[lastCf->id] == inlierIdxInCluster) {
                    prevWord = w.second;
                    break;
                }
            }
            if (prevWord) {
                // Frame
                prevWord->addDataframe(_df->id);
                _df->wordsReference.push_back(prevWord);

                //Cluster
                if (std::find(prevWord->clusters.begin(), prevWord->clusters.end(), currentCluster->id) == prevWord->clusters.end()) {
                    std::vector<float> clusterProjections = {currentCluster->featureProjections[inlierIdxInDataframe].x,
                                                            currentCluster->featureProjections[inlierIdxInDataframe].y};
                    prevWord->addClusterframe(currentCluster->id, currentCluster, inlierIdxInDataframe, clusterProjections);
                    currentCluster->addWord(prevWord);
                    
                    // 666 CHECK IF IT IS NECESARY
                    for (auto &id : prevWord->clusters) {
                        currentCluster->updateCovisibility(id);
                        // Add current cluster id to others cluster covisibility
                        mClusterframes[id]->updateCovisibility(currentCluster->id);
                    }
                }
            }
            else {
                // Create word
                int wordId = 0;
                if(mWordDictionary.size()>0)
                    wordId = mWordDictionary.rbegin()->first+1;

                auto pclPoint = (*transformedFeatureCloud)[inlierIdxInCluster];
                std::vector<float> point = {pclPoint.x, pclPoint.y, pclPoint.z};
                auto descriptor = lastCf->featureDescriptors.row(inlierIdxInCluster);

                auto newWord = std::shared_ptr<Word<PointType_>>(new Word<PointType_>(wordId, point, descriptor));

                //Add word to current dataframe
                newWord->addDataframe(_df->id);
                _df->wordsReference.push_back(newWord);

                // It happens, in cluster transition, that if the word is being created, i.e., does not
                // exist before the last pair of dataframes, it is only asigned to the new cluster. Thus,
                // need to check that and add word to both clusters. This happens always in the transition between clusters
                // Thus, if cluster's wordsReferences is empty, it is needed.
                if (isNewCluster) {
                    // Add word to new cluster (new dataframe is representative of the new cluster)
                    std::vector<float> dataframeProjections = {currentCluster->featureProjections[inlierIdxInDataframe].x, currentCluster->featureProjections[inlierIdxInDataframe].y};
                    newWord->addClusterframe(currentCluster->id, currentCluster, inlierIdxInDataframe, dataframeProjections);

                    currentCluster->updateCovisibility(lastCf->id);
                    currentCluster->updateCovisibility(currentCluster->id);
                    currentCluster->addWord(newWord);

                    // Add word to last cluster
                    std::vector<float> clusterProjections = {lastCf->featureProjections[inlierIdxInCluster].x, lastCf->featureProjections[inlierIdxInCluster].y};
                    newWord->addClusterframe(lastCf->id, lastCf, inlierIdxInCluster, clusterProjections);
                    lastCf->updateCovisibility(currentCluster->id);
                    lastCf->updateCovisibility(lastCf->id);
                    lastCf->addWord(newWord);
                }
                else {
                    // Add word to current cluster
                    std::vector<float> clusterProjections = {currentCluster->featureProjections[inlierIdxInCluster].x, currentCluster->featureProjections[inlierIdxInCluster].y};
                    newWord->addClusterframe(currentCluster->id, currentCluster, inlierIdxInCluster, clusterProjections);
                    currentCluster->updateCovisibility(currentCluster->id);
                    currentCluster->addWord(newWord);
                }
                mWordDictionary[wordId] = newWord;
            }
        }
        /*if(isNewCluster){
            std::cout << "Number of words in cluster " +std::to_string(lastCf->id) + " = " + std::to_string(lastCf->wordsReference.size()) << std::endl;
            std::cout << "Words in Dictionary " + std::to_string(mWordDictionary.size()) << std::endl;
        }*/
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::wordComparison(std::shared_ptr<mico::ClusterFrames<PointType_>> _queryCluster,
                                                    std::shared_ptr<mico::ClusterFrames<PointType_>> _trainCluster) { 
        typename pcl::PointCloud<PointType_>::Ptr transformedFeatureCloud(new pcl::PointCloud<PointType_>());
        pcl::transformPointCloud(*_trainCluster->featureCloud, *transformedFeatureCloud, _trainCluster->pose);
        
        std::vector<cv::DMatch> cvInliers = _queryCluster->multimatchesInliersCfs[_trainCluster->id];

        // Debugging purposes
        cv::Mat displayQuery = _queryCluster->left.clone();
        cv::Mat displayTrain = _trainCluster->left.clone();
        cv::Mat display;
        cv::hconcat(displayTrain, displayQuery, display);

        // Word count
        int newWords = 0;
        int duplicatedWords = 0; 
        int wordsOnlyInOneCluster =0; 
        int goodWords = 0;
        for (unsigned inlierIdx = 0; inlierIdx < cvInliers.size(); inlierIdx++) { 
            int inlierIdxInQuery = cvInliers[inlierIdx].queryIdx;
            int inlierIdxInTrain = cvInliers[inlierIdx].trainIdx;

            std::shared_ptr<Word<PointType_>> trainWord = nullptr;
            // Check if exists a word with the id of the descriptor inlier in the train cluster
            for (auto &w : _trainCluster->wordsReference)       {
                if (w.second->idxInCf[_trainCluster->id] == inlierIdxInTrain) {
                    trainWord = w.second;
                    break;
                }
            }
            // Check if exists a word with the id of the descriptor inlier in the query cluster
            std::shared_ptr<Word<PointType_>> queryWord = nullptr;
            for (auto &w : _queryCluster->wordsReference)    {
                if (w.second->idxInCf[_queryCluster->id] == inlierIdxInQuery) {
                    queryWord = w.second;
                    break;
                }
            }

            if(queryWord){
                if(trainWord){    
                    if(trainWord->id != queryWord->id){
                        // Merge words. Erase newest word
                        trainWord->mergeWord(queryWord);
                        //Delete word from mWordDictionary
                        mWordDictionary.erase(queryWord->id);
                        // Add word
                        _queryCluster->addWord(trainWord);
                        duplicatedWords++;
                    }
                    else{
                        goodWords++;
                    }
                }else{
                    // Add info of queryWord in train cluster and update queryWord
                    std::vector<float> trainProjections = {_trainCluster->featureProjections[inlierIdxInTrain].x, _trainCluster->featureProjections[inlierIdxInTrain].y};
                    queryWord->addClusterframe(_trainCluster->id, _trainCluster, inlierIdxInTrain, trainProjections);
                    _trainCluster->addWord(queryWord);

                    // Update covisibility
                    _trainCluster->updateCovisibility(_queryCluster->id);
                    _queryCluster->updateCovisibility(_trainCluster->id);

                    wordsOnlyInOneCluster++;
                }     
            }else{
                if(trainWord){
                    // Add info of trainWord in query cluster and update trainWord
                    std::vector<float> queryProjections = {_queryCluster->featureProjections[inlierIdxInQuery].x, _queryCluster->featureProjections[inlierIdxInQuery].y};
                    trainWord->addClusterframe(_queryCluster->id, _queryCluster, inlierIdxInQuery, queryProjections);
                    _queryCluster->addWord(trainWord);

                    // Update covisibility
                    _trainCluster->updateCovisibility(_queryCluster->id);
                    _queryCluster->updateCovisibility(_trainCluster->id);
                    wordsOnlyInOneCluster++;
                }else{
                    // New word
                    int wordId = mWordDictionary.rbegin()->first+1;
                    auto pclPoint = (*transformedFeatureCloud)[inlierIdxInTrain];   // 3D point of the trainCluster
                    std::vector<float> point = {pclPoint.x, pclPoint.y, pclPoint.z};
                    auto descriptor = _trainCluster->featureDescriptors.row(inlierIdxInTrain);
                    auto newWord = std::shared_ptr<Word<PointType_>>(new Word<PointType_>(wordId, point, descriptor));
                    
                    // Add word to train cluster
                    std::vector<float> trainProjections = {_trainCluster->featureProjections[inlierIdxInTrain].x, _trainCluster->featureProjections[inlierIdxInTrain].y};
                    newWord->addClusterframe(_trainCluster->id, _trainCluster, inlierIdxInTrain, trainProjections);
                    _trainCluster->updateCovisibility(_queryCluster->id);
                    _trainCluster->addWord(newWord);

                    // Add word to query cluster
                    std::vector<float> queryProjections = {_queryCluster->featureProjections[inlierIdxInQuery].x, _queryCluster->featureProjections[inlierIdxInQuery].y};
                    newWord->addClusterframe(_queryCluster->id, _queryCluster, inlierIdxInQuery, queryProjections);
                    _queryCluster->updateCovisibility(_trainCluster->id);
                    _queryCluster->addWord(newWord);

                    // Add word to dictionary
                    mWordDictionary[wordId] = newWord;

                    newWords++;
                }
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::clusterComparison(std::map<int,std::shared_ptr<ClusterFrames<PointType_>>> _clusterSubset, bool _localComparison)
    {
        for (auto queryCluster = _clusterSubset.rbegin(); queryCluster != _clusterSubset.rend(); queryCluster++){
            for (auto trainCluster = _clusterSubset.begin(); trainCluster != _clusterSubset.end() && queryCluster->second->id>trainCluster->second->id; trainCluster++){
                Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
                if (!transformationBetweenClusterframes<PointType_>(queryCluster->second, trainCluster->second, transformation,
                                                                    1, 0.03,
                                                                    1000, 10,
                                                                    25.0 /* Descriptor distance factor*/ , 1000)) //TODO: json parameters
                { 
                    std::cout << "DatabaseMarkI, <10 inliers between cluster: " + std::to_string(queryCluster->second->id) + " and cluster " +
                                                std::to_string(trainCluster->second->id) + " " << std::endl;
                }else{
                    std::cout << "DatabaseMarkI, comparison between cluster: " + std::to_string(queryCluster->second->id) + " and cluster " +
                                                std::to_string(trainCluster->second->id) << std::endl;
                    wordComparison(queryCluster->second,trainCluster->second);
                }
            }
            if(_localComparison)
                break;
        }
    }

    //--------------------------------------------------------------------------------------------------------------------- 
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_> 
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::clusterComparison(int _nCluster)  { 
        mNumCluster = _nCluster; 
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::addDataframe(std::shared_ptr<mico::DataFrame<PointType_>> _df) {
        if (this->mLastClusterframe != nullptr) {
            auto score = this->dfToClusterScore(_df);
            this->status("DatabaseMarkI", "Score: " +std::to_string(score)+ " between df: "  +std::to_string(_df->id) +  " and cluster: " + std::to_string(this->mLastClusterframe->id));
            if (score > mScore) { /// 666 Cluster creation
                // Adding df in cluster
                mLastClusterframe->addDataframe(_df);
                mLastDataFrame=_df;

                return false;
            }
            else { // NEW CLUSTER
                // Create completed cluster signature
                this->writeClusterSignature(mLastClusterframe);
                mLastClusterframe->lastTransformation = _df->lastTransformation;
                // Create cluster
                this->createCluster(_df);
                this->wordCreation(_df);
                this->mLastDataFrame=_df;
                return true;
            }
        }
        else { // INITIALIZE FIRST CLUSTER
            this->createCluster(_df);
            this->dfToClusterScore(_df);
            this->mLastDataFrame=_df;
            return true;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline std::unordered_map<int, std::shared_ptr<mico::Word<PointType_>>> DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::getDictionary() {
        return mWordDictionary;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::clusterFrames() {
        return mClusterframes;
    }

} // namespace mico 