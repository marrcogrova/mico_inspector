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
            std::cout << "\033[31m Error! Mico compiled without DBOW2, so can't use vocabulary \033[0m" << std::endl;
            return false;
        #endif
    }
    

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool DatabaseCF<PointType_, DebugLevel_, OutInterface_>::addDataframe(typename DataFrame<PointType_>::Ptr _df) {
        std::cout << "---------------------------------------" << std::endl;
        std::cout << "Added new DF" << std::endl;
        if (lastClusterframe_ == nullptr) {
            std::cout << "First CF" << std::endl;
            createCluster(_df);
        }else{
            auto score = checkSimilarity(_df);
            std::cout << "SCORE: ----- " << score <<"/" << minScore_ << std::endl;

            this->status("DatabaseCF", "Score: " +std::to_string(score)+ " between df: "  +std::to_string(_df->id) +  " and cluster: " + std::to_string(this->lastClusterframe_->id));
            
            if (score < minScore_) { // Create new cluster
                std::cout << "No similar to cluster, created new one" << std::endl;
                auto prevCluster = lastClusterframe_;   
                createCluster(_df); // Too dark... 
                wordCreation(prevCluster, lastClusterframe_);
            }else{
                std::cout << "Similar to prev cluster" << std::endl;
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
            std::cout << "\033[31m There are not enough matches between CFs, warning \033[0m" << std::endl;
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
        
        std::cout << "Number of words in cluster " +std::to_string(_prev->id) + " = " + std::to_string(_prev->wordsReference.size()) << std::endl;
        std::cout << "Words in Dictionary " + std::to_string(wordDictionary_.size()) << std::endl;

    }


} // namespace mico 