////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////



namespace rgbd{
    template<typename PointType_>
    void Database<PointType_>::addKeyframe(std::shared_ptr<Keyframe<PointType_> > &_kf) {
        mKeyframes.push_back(_kf);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::connectKeyframes(unsigned _id1, unsigned _id2){
        auto &kf = mKeyframes[_id1];
        auto &prevKf = mKeyframes[_id2];
        pcl::PointCloud<PointType_> transCloud;
        pcl::transformPointCloud(*kf->featureCloud, transCloud, kf->pose);    // 666 Transform only chosen points not all.

        for(unsigned inlierIdx = 0; inlierIdx < kf->multimatchesInliersKfs[_id2].size(); inlierIdx++){ // Assumes that is only matched with previous cloud, loops arenot handled in this method
            std::shared_ptr<Word> prevWord = nullptr;
            int inlierIdxInCurrent = kf->multimatchesInliersKfs[_id2][inlierIdx].queryIdx;
            int inlierIdxInPrev = kf->multimatchesInliersKfs[_id2][inlierIdx].trainIdx;
            for(auto &w: prevKf->wordsReference){
                if(w->idxInKf[prevKf->id] == inlierIdxInPrev){
                    prevWord = w;
                    break;
                }
            }
            if(prevWord){
                prevWord->frames.push_back(kf->id);
                prevWord->projections[kf->id] = {kf->featureProjections[inlierIdxInCurrent].x, kf->featureProjections[inlierIdxInCurrent].y};
                prevWord->idxInKf[kf->id] = inlierIdxInCurrent;
                kf->wordsReference.push_back(prevWord);
            }else{
                int wordId = mWorldDictionary.size();
                mWorldDictionary[wordId] = std::shared_ptr<Word>(new Word);
                mWorldDictionary[wordId]->id        = wordId;
                mWorldDictionary[wordId]->point     = {transCloud.at(inlierIdxInCurrent).x, transCloud.at(inlierIdxInCurrent).y, transCloud.at(inlierIdxInCurrent).z};
                mWorldDictionary[wordId]->frames    = {prevKf->id, kf->id};
                mWorldDictionary[wordId]->projections[prevKf->id] = {prevKf->featureProjections[inlierIdxInPrev].x, prevKf->featureProjections[inlierIdxInPrev].y};
                mWorldDictionary[wordId]->projections[kf->id] = {kf->featureProjections[inlierIdxInCurrent].x, kf->featureProjections[inlierIdxInCurrent].y};
                mWorldDictionary[wordId]->idxInKf[prevKf->id] = inlierIdxInPrev;
                mWorldDictionary[wordId]->idxInKf[kf->id] = inlierIdxInCurrent;
                kf->wordsReference.push_back(mWorldDictionary[wordId]);
                prevKf->wordsReference.push_back(mWorldDictionary[wordId]);
            }
        }
    }

}
