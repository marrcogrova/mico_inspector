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

#include <mico/base/map3d/Word.h>

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
            mNumCluster = (int)_configFile["clusterComparison"];
            
        #ifdef USE_DBOW2
            if(_configFile.contains("vocabulary")) 
                mVocabulary.load(_configFile["vocabulary"]);
            //mVocabulary.setScoringType(DBoW2::L2_NORM);  //TODO: Change this scoring type
            return !mVocabulary.empty();
        #else
            return false;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::addDataframe(std::shared_ptr<mico::Dataframe<PointType_>> _df) {
        if (this->mLastDataframe != nullptr) {
            auto score = this->computeScore(_df);
            this->status("DatabaseMarkI", "Score: " +std::to_string(score)+ " between df: "  +std::to_string(_df->id()) +  " and cluster: " + std::to_string(mLastDataframe->id()));
            if (score > mScore) { /// 666 Cluster creation
                return false;
            }
            else { // NEW CLUSTER
                // Create df signature
                this->computeSignature(mLastDataframe);  // 666 duplicated?
                
                // Create keyframe
                this->updateCurrentKeyframe(_df);
                this->wordCreation(_df);
                return true;
            }
        }
        else { // INITIALIZE FIRST CLUSTER
            this->updateCurrentKeyframe(_df);
            this->computeScore(_df);
            return true;
        }
    }


    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::updateCurrentKeyframe(std::shared_ptr<mico::Dataframe<PointType_>> _df) {
        
        // Update dataframe
        mDataframes[_df->id()] = _df;

        // Update MMI of previous daraframe
        // if (mDataframes.size() > 1){
        //     if(mNumCluster>1){
        //         int n = 0;
        //         // local dataframe comparison
        //         std::map<int, std::shared_ptr<Dataframe<PointType_>>> localSubset;
        //         localSubset[mLastDataframe->id()] = mLastDataframe;
        //         for (auto trainDf = mDataframes.rbegin(); trainDf != mDataframes.rend() && n <= mNumCluster +2; trainDf++, n++)
        //         {   
        //             localSubset[trainDf->first] = trainDf->second;
        //         }
        //         dfComparison(localSubset,true);
        //     }
        // }
        mLastDataframe = _df;

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline double DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::computeScore(std::shared_ptr<mico::Dataframe<PointType_>> _df) {
        #ifdef USE_DBOW2
            // Creating df signature and featVec with DBoW2
            std::vector<cv::Mat> descriptors;
            for (int r = 0; r < _df->featureDescriptors().rows; r++) {
                descriptors.push_back(_df->featureDescriptors().row(r));
            }

            // Compute signature
            DBoW2::BowVector signature;
            DBoW2::FeatureVector featVec;
            mVocabulary.transform(descriptors,signature, featVec, 4);
            _df->signature(signature);
            _df->featureVector(featVec);

            if (mLastDataframe->signature().empty()) {
                mLastDataframe->signature(signature);
            }
            // Adding df in current dataframe or create a new one
            return mVocabulary.score(_df->signature(), mLastDataframe->signature());
        #else
            return 0;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::computeSignature(std::shared_ptr<mico::Dataframe<PointType_>> &_df) {
        std::vector<cv::Mat> descriptors;
        for (auto &w : _df->words()) {
            descriptors.push_back(w.second->descriptor);
        }

        DBoW2::BowVector signature;
        DBoW2::FeatureVector featVec; 
        mVocabulary.transform(descriptors,signature, featVec, 4);
        _df->signature(signature);
        _df->featureVector(featVec);
    }

    //---------------------------------------------------------------------------------------------------------------------

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::savePoses(std::string _posesFileName)
    {
        assert(false);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::wordCreation(std::shared_ptr<mico::Dataframe<PointType_>> _df) {
        // bool isNewCluster = (mLastDataframe->words().size() == 0) && (mLastDataframe->id() != 0);
        // std::shared_ptr<Dataframe<PointType_>> lastDf;
        // if (isNewCluster){
        //     // 666 This only works if dataframes are sequential, it might be good if there is a way to identify previous dataframes
        //     // lastDf = mDataframes[mLastDataframe->id() - 1];
        //     lastDf = (++mDataframes.rbegin())->second;
        // }
        // else {
        //     lastDf = mLastDataframe;
        // }
        // auto currentDf = mLastDataframe;

        auto currentDf = mLastDataframe;
        auto lastDf = (++mDataframes.rbegin())->second;

        typename pcl::PointCloud<PointType_>::Ptr transformedFeatureCloud(new pcl::PointCloud<PointType_>());
        pcl::transformPointCloud(*lastDf->featureCloud(), *transformedFeatureCloud, lastDf->pose());
        std::vector<cv::DMatch> cvInliers = currentDf->crossReferencedInliers()[lastDf->id()];

        for (unsigned inlierIdx = 0; inlierIdx < cvInliers.size(); inlierIdx++){
            // 666 Assumes that is only matched with previous cloud, loops are not handled in this method
            std::shared_ptr<Word<PointType_>> prevWord = nullptr;
            int inlierIdxInDataframe = cvInliers[inlierIdx].queryIdx;
            int inlierIdxInCluster = cvInliers[inlierIdx].trainIdx;

            // Check if exists a word with the id of the descriptor inlier
            for (auto &w : /*mWordDictionary*/ lastDf->words()){ // TODO: Can we check if the word have current dataframe id as key?
                if (w.second->idxInDf[lastDf->id()] == inlierIdxInCluster) {
                    prevWord = w.second;
                    break;
                }
            }
            if (prevWord) {
                //Cluster
                if (std::find(prevWord->dfIds.begin(), prevWord->dfIds.end(), currentDf->id()) == prevWord->dfIds.end()) {
                    std::vector<float> projection = {   currentDf->featureProjections()[inlierIdxInDataframe].x,
                                                        currentDf->featureProjections()[inlierIdxInDataframe].y};
                    prevWord->addObservation(currentDf->id(), currentDf, inlierIdxInDataframe, projection);
                    currentDf->addWord(prevWord);
                    
                    // 666 CHECK IF IT IS NECESARY
                    for (auto &id : prevWord->dfIds) {
                        currentDf->appendCovisibility(id);
                        // Add current dataframe id to others dataframe covisibility
                        mDataframes[id]->appendCovisibility(currentDf->id());
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
                auto descriptor = lastDf->featureDescriptors().row(inlierIdxInCluster);

                auto newWord = std::shared_ptr<Word<PointType_>>(new Word<PointType_>(wordId, point, descriptor));

                //Add word to current dataframe
                currentDf->addWord(newWord);

                // Add word to new dataframe (new dataframe is representative of the new dataframe)
                std::vector<float> dataframeProjections = { currentDf->featureProjections()[inlierIdxInDataframe].x, 
                                                            currentDf->featureProjections()[inlierIdxInDataframe].y};
                newWord->addObservation(currentDf->id(), currentDf, inlierIdxInDataframe, dataframeProjections);

                currentDf->appendCovisibility(lastDf->id());
                currentDf->addWord(newWord);

                // Add word to last dataframe
                std::vector<float> projection = {   lastDf->featureProjections()[inlierIdxInCluster].x, 
                                                    lastDf->featureProjections()[inlierIdxInCluster].y};
                newWord->addObservation(lastDf->id(), lastDf, inlierIdxInCluster, projection);
                lastDf->appendCovisibility(lastDf->id());
                lastDf->addWord(newWord);
                
                mWordDictionary[wordId] = newWord;
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::wordComparison(
                                                    std::shared_ptr<mico::Dataframe<PointType_>> _queryDf,
                                                    std::shared_ptr<mico::Dataframe<PointType_>> _trainDf) { 
        typename pcl::PointCloud<PointType_>::Ptr transformedFeatureCloud(new pcl::PointCloud<PointType_>());
        pcl::transformPointCloud(*_trainDf->featureCloud(), *transformedFeatureCloud, _trainDf->pose());
        
        std::vector<cv::DMatch> cvInliers = _queryDf->crossReferencedInliers()[_trainDf->id()];

        // Word count
        int newWords = 0;
        int duplicatedWords = 0; 
        int wordsOnlyInOneCluster =0; 
        int goodWords = 0;
        for (unsigned inlierIdx = 0; inlierIdx < cvInliers.size(); inlierIdx++) { 
            int inlierIdxInQuery = cvInliers[inlierIdx].queryIdx;
            int inlierIdxInTrain = cvInliers[inlierIdx].trainIdx;

            std::shared_ptr<Word<PointType_>> trainWord = nullptr;
            // Check if exists a word with the id of the descriptor inlier in the train dataframe
            for (auto &w : _trainDf->words())       {
                if (w.second->idxInDf[_trainDf->id()] == inlierIdxInTrain) {
                    trainWord = w.second;
                    break;
                }
            }
            // Check if exists a word with the id of the descriptor inlier in the query dataframe
            std::shared_ptr<Word<PointType_>> queryWord = nullptr;
            for (auto &w : _queryDf->words())    {
                if (w.second->idxInDf[_queryDf->id()] == inlierIdxInQuery) {
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
                        _queryDf->addWord(trainWord);
                        duplicatedWords++;
                    }
                    else{
                        goodWords++;
                    }
                }else{
                    // Add info of queryWord in train dataframe and update queryWord
                    std::vector<float> trainProjections = { _trainDf->featureProjections()[inlierIdxInTrain].x, 
                                                            _trainDf->featureProjections()[inlierIdxInTrain].y};
                    queryWord->addObservation(_trainDf->id(), _trainDf, inlierIdxInTrain, trainProjections);
                    _trainDf->addWord(queryWord);

                    // Update covisibility
                    _trainDf->appendCovisibility(_queryDf->id());
                    _queryDf->appendCovisibility(_trainDf->id());

                    wordsOnlyInOneCluster++;
                }     
            }else{
                if(trainWord){
                    // Add info of trainWord in query dataframe and update trainWord
                    std::vector<float> queryProjections = { _queryDf->featureProjections()[inlierIdxInQuery].x, 
                                                            _queryDf->featureProjections()[inlierIdxInQuery].y};
                    trainWord->addObservation(_queryDf->id(), _queryDf, inlierIdxInQuery, queryProjections);
                    _queryDf->addWord(trainWord);

                    // Update covisibility
                    _trainDf->appendCovisibility(_queryDf->id());
                    _queryDf->appendCovisibility(_trainDf->id());
                    wordsOnlyInOneCluster++;
                }else{
                    // New word
                    int wordId = mWordDictionary.rbegin()->first+1;
                    auto pclPoint = (*transformedFeatureCloud)[inlierIdxInTrain];   // 3D point of the trainDf
                    std::vector<float> point = {pclPoint.x, pclPoint.y, pclPoint.z};
                    auto descriptor = _trainDf->featureDescriptors().row(inlierIdxInTrain);
                    auto newWord = std::shared_ptr<Word<PointType_>>(new Word<PointType_>(wordId, point, descriptor));
                    
                    // Add word to train dataframe
                    std::vector<float> trainProjections = { _trainDf->featureProjections()[inlierIdxInTrain].x, 
                                                            _trainDf->featureProjections()[inlierIdxInTrain].y};
                    newWord->addObservation(_trainDf->id(), _trainDf, inlierIdxInTrain, trainProjections);
                    _trainDf->appendCovisibility(_queryDf->id());
                    _trainDf->addWord(newWord);

                    // Add word to query dataframe
                    std::vector<float> queryProjections = { _queryDf->featureProjections()[inlierIdxInQuery].x, 
                                                            _queryDf->featureProjections()[inlierIdxInQuery].y};
                    newWord->addObservation(_queryDf->id(), _queryDf, inlierIdxInQuery, queryProjections);
                    _queryDf->appendCovisibility(_trainDf->id());
                    _queryDf->addWord(newWord);

                    // Add word to dictionary
                    mWordDictionary[wordId] = newWord;

                    newWords++;
                }
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::dfComparison(std::map<int,std::shared_ptr<Dataframe<PointType_>>> _dfSet, bool _localComparison)
    {
        for (auto queryDf = _dfSet.rbegin(); queryDf != _dfSet.rend(); queryDf++){
            for (auto trainDf = _dfSet.begin(); (trainDf != _dfSet.end() && (queryDf->first) > (trainDf->first)); trainDf++){
                Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
                if (!transformationBetweenFeatures<PointType_>( queryDf->second, 
                                                                trainDf->second, 
                                                                transformation,
                                                                1, 
                                                                0.03,
                                                                1000, 
                                                                10,
                                                                25.0 /* Descriptor distance factor*/ , 
                                                                10)) //TODO: json parameters
                { 
                    std::cout << "DatabaseMarkI, <10 inliers between df: " + std::to_string(queryDf->first) + " and df " +
                                                std::to_string(trainDf->first) + " " << std::endl;
                }else{
                    std::cout << "DatabaseMarkI, comparison between df: " + std::to_string(queryDf->first) + " and df " +
                                                std::to_string(trainDf->first) << std::endl;
                    wordComparison(queryDf->second,trainDf->second);
                }
            }
            if(_localComparison)
                break;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline std::unordered_map<int, std::shared_ptr<mico::Word<PointType_>>> DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::dictionary() {
        return mWordDictionary;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline std::map<int, std::shared_ptr<Dataframe<PointType_>>> DatabaseMarkI<PointType_, DebugLevel_, OutInterface_>::dataframes() {
        return mDataframes;
    }

} // namespace mico 