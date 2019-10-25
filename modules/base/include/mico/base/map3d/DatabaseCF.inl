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
    inline bool DatabaseCF<PointType_, DebugLevel_, OutInterface_>::init(const cjson::Json &_configFile) {
        minScore_ = (double)_configFile["similarity_score"];
        
        #ifdef USE_DBOW2
            if(_configFile.contains("vocabulary")) 
                vocabulary_.load(_configFile["vocabulary"]);
            //mVocabulary.setScoringType(DBoW2::L2_NORM);  //TODO: Change this scoring type
            return !vocabulary_.empty();
        #else
            this->error("DatabaseCF","Error! Mico compiled without DBOW2, so can't use vocabulary");
            return false;
        #endif
    }
    

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseCF<PointType_, DebugLevel_, OutInterface_>::addDataframe(typename DataFrame<PointType_>::Ptr _df) {
        if (lastClusterframe_ == nullptr) {
            createCluster(_df);
        }else{
            auto score = checkSimilarity(_df);
            
            this->status("DatabaseCF", "Score: " +std::to_string(score)+ " between df: "  +std::to_string(_df->id) +  " and cluster: " + std::to_string(this->lastClusterframe_->id));
            
            if (score < minScore_) { // Create new cluster
                auto prevCluster = lastClusterframe_;   
                createCluster(_df); // Too dark... 
                wordCreation(prevCluster, lastClusterframe_);

                if(clusterframes_.size() > localComparisonSize_){  /// 666 Parametrize
                    unsigned int n = 0;
                    // local cluster comparison
                    std::map<int, std::shared_ptr<ClusterFrames<PointType_>>> localClusterSubset;
                    for (   auto trainCluster = clusterframes_.rbegin(); 
                            trainCluster != clusterframes_.rend() && n <= localComparisonSize_; 
                            trainCluster++, n++)
                    {   
                        localClusterSubset[trainCluster->first] = trainCluster->second;
                    }
                    clusterComparison(localClusterSubset,true);
                }
            }else{
                return false;   // No CF created, return false
            }
        }

        return true;    // CF created, return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline typename ClusterFrames<PointType_>::Ptr DatabaseCF<PointType_, DebugLevel_, OutInterface_>::lastCluster() const{
        return lastClusterframe_;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseCF<PointType_, DebugLevel_, OutInterface_>::createCluster(typename DataFrame<PointType_>::Ptr _df) {
        // Create cluster
        int id = clusterframes_.size();
        typename ClusterFrames<PointType_>::Ptr cluster = typename ClusterFrames<PointType_>::Ptr(new ClusterFrames<PointType_>(_df, id));

        // Compute cluster signature
        computeSignature(cluster);

        // Update cluster
        clusterframes_[id] = cluster;
        lastClusterframe_ = cluster;
    }

    //---------------------------------------------------------------------------------------------------------------------

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseCF<PointType_, DebugLevel_, OutInterface_>::computeSignature(typename mico::ClusterFrames<PointType_>::Ptr &_cluster) {
        #ifdef USE_DBOW2
            std::vector<cv::Mat> descriptors;
            for (auto &w : _cluster->wordsReference) {
                descriptors.push_back(w.second->descriptor);
            }
            vocabulary_.transform(descriptors, _cluster->signature, _cluster->featVec, 4);
        #endif
    }


    //---------------------------------------------------------------------------------------------------------------------

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline double DatabaseCF<PointType_, DebugLevel_, OutInterface_>::checkSimilarity(typename DataFrame<PointType_>::Ptr _df) {
        #ifdef USE_DBOW2
            // Creating df signature and featVec with DBoW2
            std::vector<cv::Mat> descriptors;
            for (int r = 0; r < _df->featureDescriptors.rows; r++) {
                descriptors.push_back(_df->featureDescriptors.row(r));
            }
            vocabulary_.transform(descriptors, _df->signature, _df->featVec, 4);
            if (lastClusterframe_->signature.empty()) {
                lastClusterframe_->signature = _df->signature;
            }
            // Adding df in current cluster or create a new one
            double score = vocabulary_.score(_df->signature, lastClusterframe_->signature);
            return score;
        #else
            return 0;
        #endif
    }


    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseCF<PointType_, DebugLevel_, OutInterface_>::wordCreation(typename ClusterFrames<PointType_>::Ptr _prev, typename ClusterFrames<PointType_>::Ptr _current) {
        // Compute inliers between CFs which will be promoted to be words
        Eigen::Matrix4f T;
        if(!transformationBetweenClusterframes(_prev, _current, T)){
            this->error("DatabaseCF","There are not enough matches between CFs, warning");
            return;
        }
        std::vector<cv::DMatch> cvInliers = _current->multimatchesInliersCfs[_prev->id];

        // Transform cloud to get an estimate of the 3d position of words.
        typename pcl::PointCloud<PointType_>::Ptr transformedFeatureCloud(new pcl::PointCloud<PointType_>());
        pcl::transformPointCloud(*_prev->featureCloud, *transformedFeatureCloud, _prev->pose);

        // Iterate over inliers
        for (unsigned inlierIdx = 0; inlierIdx < cvInliers.size(); inlierIdx++){
            std::shared_ptr<Word<PointType_>> prevWord = nullptr;
            int inlierIdxPrev = cvInliers[inlierIdx].queryIdx;
            int inlierIdxCurr = cvInliers[inlierIdx].trainIdx;

            // Check if exists a word with the id of the descriptor inlier
            for (auto &w : _prev->wordsReference){ // TODO: Can we make it faster?
                if (w.second->idxInCf[_prev->id] == inlierIdxPrev) { // 666 check if right
                    prevWord = w.second;
                    break;
                }
            }
            
            if (prevWord) {  // Fill data from new observation
                std::vector<float> projection = {   _current->featureProjections[inlierIdxCurr].x,
                                                    _current->featureProjections[inlierIdxCurr].y};
                prevWord->addClusterframe(_current->id, _current, inlierIdxCurr, projection);
                _current->addWord(prevWord);
                for (auto &id : prevWord->clusters) { // Add current cluster id to others cluster covisibility
                    _current->updateCovisibility(id);
                    clusterframes_[id]->updateCovisibility(_current->id);
                }
            }
            else { // Create new word
                int wordId = wordDictionary_.size();
                
                auto pclPoint = (*transformedFeatureCloud)[inlierIdxPrev];
                std::vector<float> point = {pclPoint.x, pclPoint.y, pclPoint.z};
                auto descriptor = _prev->featureDescriptors.row(inlierIdxCurr);

                auto newWord = std::shared_ptr<Word<PointType_>>(new Word<PointType_>(wordId, point, descriptor));

                // Add word to current cluster
                std::vector<float> clusterProjections = {_current->featureProjections[inlierIdxCurr].x, _current->featureProjections[inlierIdxCurr].y};
                newWord->addClusterframe(_current->id, _current, inlierIdxCurr, clusterProjections);
                _current->updateCovisibility(_current->id);
                _current->addWord(newWord);
 
                wordDictionary_[wordId] = newWord;
            }
        }
    }


    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseCF<PointType_, DebugLevel_, OutInterface_>::clusterComparison(std::map<int,std::shared_ptr<ClusterFrames<PointType_>>> _clusterSubset, bool _localComparison)
    {
        for (auto queryCluster = _clusterSubset.rbegin(); queryCluster != _clusterSubset.rend(); queryCluster++){
            for (auto trainCluster = _clusterSubset.begin(); trainCluster != _clusterSubset.end() && queryCluster->second->id>trainCluster->second->id; trainCluster++){
                Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
                if (!transformationBetweenClusterframes<PointType_>(queryCluster->second, trainCluster->second, transformation,
                                                                    1, 0.03,
                                                                    1000, 10,
                                                                    25.0 /* Descriptor distance factor*/ , 1000)) //TODO: json parameters
                { 
                    this->error("DatabaseCF","Database, <10 inliers between cluster: " + 
                                                std::to_string(queryCluster->second->id) + " and cluster " +
                                                std::to_string(trainCluster->second->id));
                }else{
                    this->status("DatabaseCF","Database, comparison between cluster: " + 
                                                std::to_string(queryCluster->second->id) + " and cluster " +
                                                std::to_string(trainCluster->second->id));
                    wordComparison(queryCluster->second,trainCluster->second);
                }
            }
            if(_localComparison)
                break;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseCF<PointType_, DebugLevel_, OutInterface_>::wordComparison(std::shared_ptr<mico::ClusterFrames<PointType_>> _queryCluster,
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
                        //Delete word from wordDictionary_
                        wordDictionary_.erase(queryWord->id);
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
                    int wordId = wordDictionary_.rbegin()->first+1;
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
                    wordDictionary_[wordId] = newWord;

                    newWords++;
                }
            }
        }
    }


} // namespace mico 