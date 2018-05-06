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


#include <Eigen/Eigen>

#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
#include <algorithm>

namespace rgbd{
    template<typename PointType_>
    bool Database<PointType_>::addDataframe(std::shared_ptr<DataFrame<PointType_> > &_kf,double _mk_nearest_neighbors,double _mRansacMaxDistance,int _mRansacIterations,int _mRansacMinInliers,double _mFactorDescriptorDistance) {

        //if(mDataframes.size() > 10){
        //    for(auto & w:mWordDictionary){
        //        displayWord(w.first);
        //    }
        //}

        std::vector<cv::Mat> descriptors;
        for(unsigned  r = 0; r < _kf->featureDescriptors.rows; r++){
            descriptors.push_back(_kf->featureDescriptors.row(r));
        }
        mVocabulary.transform(descriptors, _kf->signature,_kf->featVec,4);
        mDataframes[_kf->id] = _kf;

        if(mClustersMap.size() == 0){
            std::shared_ptr<ClusterFrames<PointType_>> cluster = std::shared_ptr<ClusterFrames<PointType_>>(new ClusterFrames<PointType_>);
            cluster->id=0;
            mClustersMap[cluster->id] = cluster;
            mClustersMap[0]->frames.push_back(_kf);
            mLastCluster=cluster;
        }else{
            // Connect Kf with previous frame and create seq of words
            connectKeyframes(_kf->id-1, _kf->id);
            for(unsigned i = _kf->id; i > 0; i--){
                displaySharedWords(i-1, i);
            }
            // Compare Kfs
            double score = mVocabulary.score(_kf->signature, mClustersMap[mLastCluster->id]->frames[0]->signature);
            std::cout << "Score between frame " << _kf->id << " and " << mClustersMap[mLastCluster->id]->frames[0]->id << ": " << score << std::endl;
            if(score > dbow2Score){ // 666 CHECK PARAM!!
                // Word creation
                //Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
                //computeMultiMatchesInliers(mLastCluster,_kf,transformation,_mk_nearest_neighbors,_mRansacMaxDistance,_mRansacIterations,_mRansacMinInliers,_mFactorDescriptorDistance);
                //continuousWordCreation(_kf);
                mLastCluster->frames.push_back(_kf);
                return true;
            }else{
                std::shared_ptr<ClusterFrames<PointType_>> cluster = std::shared_ptr<ClusterFrames<PointType_>>(new ClusterFrames<PointType_>);
                cluster->id=mLastCluster->id+1;
                cluster->frames.push_back(_kf);
                mClustersMap[cluster->id] = cluster;
                mLastCluster=cluster;
                return true;
            }
        }
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::connectKeyframes(unsigned _id1, unsigned _id2, bool _has3D){
        auto &kf = mDataframes[_id1];
        auto &prevKf = mDataframes[_id2];
        pcl::PointCloud<PointType_> transCloud;
        if(_has3D){
            pcl::transformPointCloud(*kf->featureCloud, transCloud, kf->pose);    // 666 Transform only chosen points not all.
        }

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
                if(!_has3D){    // 666 RETORE! needed for monocular
//                    auto triangulatedPoint = triangulateFromProjections(prevWord->projections);
//                    prevWord->point = { triangulatedPoint(0), triangulatedPoint(1), triangulatedPoint(2) };
//                    if(mWordDictionary.size() < prevWord->id+1){
//                        PointType_ p;
//                        p.x = prevWord->point[0];
//                        p.y = prevWord->point[1];
//                        p.z = prevWord->point[2];
//                        mWordDictionary.push_back(p);
//                    }else{
//                        PointType_ p;
//                        p.x = prevWord->point[0];
//                        p.y = prevWord->point[1];
//                        p.z = prevWord->point[2];
//                        mWordDictionary[prevWord->id] = p;
//                    }
                }
            }else{
                int wordId = mWordDictionary.size();
                mWordDictionary[wordId] = std::shared_ptr<Word>(new Word);
                mWordDictionary[wordId]->id        = wordId;
                if(_has3D){
                    mWordDictionary[wordId]->point     = {transCloud.at(inlierIdxInCurrent).x, transCloud.at(inlierIdxInCurrent).y, transCloud.at(inlierIdxInCurrent).z};
                    PointType_ p;
                    p.x = mWordDictionary[wordId]->point[0];
                    p.y = mWordDictionary[wordId]->point[1];
                    p.z = mWordDictionary[wordId]->point[2];
                    mWordMap.push_back(p);
                }
                mWordDictionary[wordId]->frames    = {prevKf->id, kf->id};
                mWordDictionary[wordId]->projections[prevKf->id] = {prevKf->featureProjections[inlierIdxInPrev].x, prevKf->featureProjections[inlierIdxInPrev].y};
                mWordDictionary[wordId]->projections[kf->id] = {kf->featureProjections[inlierIdxInCurrent].x, kf->featureProjections[inlierIdxInCurrent].y};
                mWordDictionary[wordId]->idxInKf[prevKf->id] = inlierIdxInPrev;
                mWordDictionary[wordId]->idxInKf[kf->id] = inlierIdxInCurrent;
                kf->wordsReference.push_back(mWordDictionary[wordId]);
                prevKf->wordsReference.push_back(mWordDictionary[wordId]);
            }
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::reset() {
        mClustersMap.clear();
        mDataframes.clear();
        mWordDictionary.clear();
        mWordMap.clear();
    }

    template<typename PointType_>
    bool Database<PointType_>::initVocabulary(const std::string &_path){
        #ifdef USE_DBOW2
            mVocabulary.load(_path);
            return !mVocabulary.empty();
        #else
            return false;
        #endif
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    Eigen::Vector3f Database<PointType_>::triangulateFromProjections(std::unordered_map<int, std::vector<float> > _projections) {
        // Compute 3D Pose 666 TODO
        typedef dlib::vector<double,3> Vect3f;
        typedef dlib::matrix<double,3,3> Matr3f;
        // Define epipolar lines of each projection.
        std::vector<Vect3f> epiInits;
        std::vector<Vect3f> epiVecs;
        for(auto &pairProj:_projections){
            int id = pairProj.first;   // 666 might be good to use maps to for keyframes list
            auto kf = mDataframes[id];
            Vect3f init = {kf->position(0), kf->position(1), kf->position(2)};
            Eigen::Matrix3f rotationEigen(kf->orientation);
            Matr3f rotation;
            for(unsigned i = 0; i < 3; i++){
                for(unsigned j = 0; j < 3; j++){
                    rotation(i,j) = rotationEigen(i,j);
                }
            }


            cv::Mat intrinsic = kf->intrinsic;
            Vect3f vector = {pairProj.second[0] - intrinsic.at<float>(0,2),
                               pairProj.second[1] - intrinsic.at<float>(1,2),
                               (intrinsic.at<float>(0,0) + intrinsic.at<float>(1,1))/2};

            vector = rotation*vector;

            epiInits.push_back(init);
            epiVecs.push_back(vector);
        }
        // Define objective function, i.e., minimize distance between epipolar lines.
                auto pointToLine2 = [](Vect3f _point, Vect3f _p1, Vect3f _p2)->double{
                    return pow(((_p2-_p1).cross(_p1-_point)).length()/(_p2-_p1).length(),2);
                };

                auto objective2Fun = [&](Vect3f _point){
                    double distance = 0;
                    for(unsigned i = 0; i < epiInits.size(); i++){
                        distance += pointToLine2(_point, epiInits[i], epiInits[i] + epiVecs[i]);
                    }
                    return distance;
                };

        // Perform minimization.
        Vect3f initPoint = epiInits[0];
        dlib::find_min_using_approximate_derivatives( dlib::bfgs_search_strategy(),
                        dlib::objective_delta_stop_strategy(1e-7),
                        objective2Fun, initPoint, -1);


        Eigen::Vector3f result = {(float) initPoint(0),(float) initPoint(1),(float) initPoint(2)};
        return result;
    }
    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::changeRelations(int id, int mate, double affinity) {
        mClustersMap[id]->relations[mate]=affinity;
        mClustersMap[mate]->relations[id]=affinity;
    }

//    //-----------------------------------------------------------------------------------------------------------------
//    template<typename PointType_>
//    void Database<PointType_>::sequentialWordCreation() {
//        auto writeWord = [&] (std::shared_ptr<DataFrame<PointType_>> trainFrame,std::shared_ptr<DataFrame<PointType_>> queryFrame, int TrainMatch, int QueryMatch) {
//            std::shared_ptr<Word> nWord = std::shared_ptr<Word>(new Word);
//            nWord->id=mWordDictionary.size() ;
//            nWord->frames.push_back(trainFrame->id);
//            nWord->frames.push_back(queryFrame->id);
//            nWord->clusters.push_back(mLastCluster->id-1);
//            nWord->projections[trainFrame->id]={trainFrame->featureProjections[TrainMatch].x,trainFrame->featureProjections[TrainMatch].y};
//            nWord->projections[queryFrame->id]={queryFrame->featureProjections[QueryMatch].x,queryFrame->featureProjections[QueryMatch].y};
//            auto pclPoint = (*trainFrame->featureCloud)[TrainMatch];
//            //pclPoint = aFrame->featureCloud->points[TMatch];
//            nWord->point={pclPoint.x,pclPoint.y,pclPoint.z};
//            nWord->idxInKf[trainFrame->id]=TrainMatch;
//            nWord->idxInKf[queryFrame->id]=QueryMatch;
//            mWordDictionary[nWord->id]=nWord;
//            mLastWord=nWord;
//            trainFrame->wordsReference.push_back(mLastWord);
//            queryFrame->wordsReference.push_back(mLastWord);
//            mClustersMap[mLastCluster->id-1]->ClusterWords[nWord->id]=nWord;
//        };
//        auto clusterFrames= mClustersMap[mLastCluster->id-1]->frames;
//        int firstFrameId=clusterFrames.front()->id;
//        // For each frame of the cluster (Vector)
//        for(auto &frame: clusterFrames){
//            // For each inlier between frames (Map)
//            for(auto &MMI:frame->multimatchesInliersKfs){
//                // Checking inliers with previous frame and not have in consideration inliers with previous cluster
//                if(MMI.first<frame->id && !mClustersMap[mLastCluster->id-1]->isFirst(frame->id)){
//                    for(auto &Inlier:MMI.second){
//                        bool found=false;
//                        // For each word in previous frame
//                        for(auto &word: clusterFrames[MMI.first-firstFrameId]->wordsReference){
//                            if(word->idxInKf[MMI.first] == Inlier.trainIdx){
//                                word->frames.push_back(frame->id);
//                                word->projections[frame->id]={frame->featureProjections[Inlier.queryIdx].x,frame->featureProjections[Inlier.queryIdx].y};
//                                word->idxInKf[frame->id]=Inlier.queryIdx;
//                                frame->wordsReference.push_back(word);
//                                found=true;
//                                break;
//                            }
//                        }
//                        if(!found){
//                            writeWord(clusterFrames[MMI.first-firstFrameId],clusterFrames[frame->id-firstFrameId],Inlier.trainIdx,Inlier.queryIdx);
//                        }
//                    }
//                }
//            }
//        }
//    }

//    //-----------------------------------------------------------------------------------------------------------------
//    template<typename PointType_>
//    void Database<PointType_>::totalWordCreation() {
//        auto writeWord = [&] (std::shared_ptr<DataFrame<PointType_>> trainFrame,std::shared_ptr<DataFrame<PointType_>> queryFrame, int TrainMatch, int QueryMatch) {
//            std::shared_ptr<Word> nWord = std::shared_ptr<Word>(new Word);
//            nWord->id=mWordDictionary.size() ;
//            nWord->frames.push_back(trainFrame->id);
//            nWord->frames.push_back(queryFrame->id);
//            nWord->clusters.push_back(mLastCluster->id-1);
//            nWord->projections[trainFrame->id]={trainFrame->featureProjections[TrainMatch].x,trainFrame->featureProjections[TrainMatch].y};
//            nWord->projections[queryFrame->id]={queryFrame->featureProjections[QueryMatch].x,queryFrame->featureProjections[QueryMatch].y};
//            auto pclPoint = (*trainFrame->featureCloud)[TrainMatch];
//            //pclPoint = aFrame->featureCloud->points[TMatch];
//            nWord->point={pclPoint.x,pclPoint.y,pclPoint.z};
//            nWord->idxInKf[trainFrame->id]=TrainMatch;
//            nWord->idxInKf[queryFrame->id]=QueryMatch;
//            mWordDictionary[nWord->id]=nWord;
//            mLastWord=nWord;
//            trainFrame->wordsReference.push_back(mLastWord);
//            queryFrame->wordsReference.push_back(mLastWord);
//            mClustersMap[mLastCluster->id-1]->ClusterWords[nWord->id]=nWord;
//        };
//        auto clusterFrames= mClustersMap[mLastCluster->id-1]->frames;
//        int firstFrameId=clusterFrames.front()->id;
//        //for(auto &frame: clusterFrames){
//        for(auto frame = clusterFrames.begin(); frame!=clusterFrames.end(); frame++){
//            for(auto &MMI: (*frame)->multimatchesInliersKfs){  //frame->multimatchesInliersKfs){
//                // Checking inliers with posterior frames and not have in consideration inliers with posterior cluster
//                if(MMI.first>(*frame)->id  && !mClustersMap[mLastCluster->id-1]->isLast((*frame)->id)){ //&& abs(MMI.first-frame->id)>1
//                    for(auto &Inlier:MMI.second){
//                        bool found=false;
//                        // Search in cluster wordReference
//                        //for(auto &word: clusterFrames[frame->id-firstFrameId]->wordsReference){
//                        for(auto &word: mClustersMap[mLastCluster->id-1]->ClusterWords){
//                            // Find key of current frame in wordReference
//                            if(word.second->idxInKf.find((*frame)->id)!=word.second->idxInKf.end()){
//                                // Compare wordReference idx with current frame idx
//                                if(word.second->idxInKf[(*frame)->id] == Inlier.queryIdx){
//                                    // Find key of matched frame in wordReference
//                                    if(word.second->idxInKf.find(MMI.first)!=word.second->idxInKf.end()){
//                                        // Compare wordReference idx with matched frame idx
//                                        if(word.second->idxInKf[MMI.first] != Inlier.trainIdx){
//                                            // Querymatch associated with more than one trainmatch
//                                        }
//                                        // Word already registered in both frames
//                                    }else{  // New word information
//                                        word.second->frames.push_back(MMI.first);
//                                        auto targetFrame = (*(frame+(MMI.first-(*frame)->id)));
//                                        word.second->projections[MMI.first]=    {   targetFrame->featureProjections[Inlier.trainIdx].x,
//                                                                                    targetFrame->featureProjections[Inlier.trainIdx].y
//                                                                                };
//                                        word.second->idxInKf[MMI.first]=Inlier.trainIdx;
//                                        (*(frame+(MMI.first-(*frame)->id)))->wordsReference.push_back(word.second);
//                                    }
//                                    found=true;
//                                    break;
//                                }
//                            }
//                        }
//                        if(!found){
//                            writeWord(*(frame+(MMI.first-(*frame)->id)),(*frame),Inlier.trainIdx,Inlier.queryIdx);
//                        }
//                    }
//                }
//            }
//        }
//    }
//    //-----------------------------------------------------------------------------------------------------------------
//    template<typename PointType_>
//    void Database<PointType_>::continuousWordCreation(std::shared_ptr<DataFrame<PointType_>> _lastKf) {
//        auto writeWord = [&] (std::shared_ptr<DataFrame<PointType_>> trainFrame,std::shared_ptr<DataFrame<PointType_>> queryFrame, int TrainMatch, int QueryMatch) {
//            std::shared_ptr<Word> nWord = std::shared_ptr<Word>(new Word);
//            nWord->id=mWordDictionary.size() ;
//            nWord->frames.push_back(trainFrame->id);
//            nWord->frames.push_back(queryFrame->id);
//            nWord->clusters.push_back(mLastCluster->id);
//            nWord->projections[trainFrame->id]={trainFrame->featureProjections[TrainMatch].x,trainFrame->featureProjections[TrainMatch].y};
//            nWord->projections[queryFrame->id]={queryFrame->featureProjections[QueryMatch].x,queryFrame->featureProjections[QueryMatch].y};
//            auto pclPoint = (*trainFrame->featureCloud)[TrainMatch];
//            //pclPoint = aFrame->featureCloud->points[TMatch];
//            nWord->point={pclPoint.x,pclPoint.y,pclPoint.z};
//            nWord->idxInKf[trainFrame->id]=TrainMatch;
//            nWord->idxInKf[queryFrame->id]=QueryMatch;
//            mWordDictionary[nWord->id]=nWord;
//            mLastWord=nWord;
//            trainFrame->wordsReference.push_back(mLastWord);
//            queryFrame->wordsReference.push_back(mLastWord);
//            mLastCluster->ClusterWords[nWord->id]=nWord;
//        };
//        auto clusterFrames= mLastCluster->frames;
//        for(auto &MMI: _lastKf->multimatchesInliersKfs){
//            // Checking inliers with previous frames and not have in consideration inliers with posterior cluster
//            for(auto &Inlier:MMI.second){
//                bool found=false;
//                // Search in cluster wordReference
//                for(auto &word: mLastCluster->ClusterWords){
//                    if(word.second->idxInKf.find(MMI.first)!=word.second->idxInKf.end()){
//                        if(word.second->idxInKf[MMI.first] == Inlier.trainIdx){
//                            if(word.second->idxInKf.find(_lastKf->id)!=word.second->idxInKf.end()){
//                                if(word.second->idxInKf[_lastKf->id] =! Inlier.queryIdx){
//                                    // Querymatch associated with more than one trainmatch
//                                }
//                                // Word already registered in both frames
//                            }else{  // New word information
//                                word.second->frames.push_back(_lastKf->id);
//                                word.second->projections[_lastKf->id]=    {   _lastKf->featureProjections[Inlier.queryIdx].x,
//                                                                              _lastKf->featureProjections[Inlier.queryIdx].y
//                                                                          };
//                                word.second->idxInKf[_lastKf->id]=Inlier.queryIdx;
//                                _lastKf->wordsReference.push_back(word.second);
//                            }
//                            found=true;
//                            break;
//                        }
//                    }
//                }
//                if(!found){
//                    writeWord(clusterFrames[MMI.first-clusterFrames.front()->id],_lastKf,Inlier.trainIdx,Inlier.queryIdx);
//                }
//            }

//        }
//    }

//    //-----------------------------------------------------------------------------------------------------------------
//    template<typename PointType_>
//    void Database<PointType_>::computeMultiMatchesInliers(std::shared_ptr<ClusterFrames<PointType_>> _targetCluster,std::shared_ptr<DataFrame<PointType_> > &_lastKf,Eigen::Matrix4f &_transformation,double _mk_nearest_neighbors,double _mRansacMaxDistance,int _mRansacIterations,int _mRansacMinInliers,double _mFactorDescriptorDistance) {
//        for(auto queryKf = _targetCluster->frames.begin() ; queryKf != _targetCluster->frames.end() ; queryKf++){
//            // Dont recompute inliers between frames
//            if(abs((*queryKf)->id-_lastKf->id)>1){ //queryKf-targetCluster->frames.begin()!=trainKf-targetCluster->frames.begin()
//                transformationBetweenFeatures( *queryKf, _lastKf, _transformation,_mk_nearest_neighbors,_mRansacMaxDistance,_mRansacIterations,_mRansacMinInliers,_mFactorDescriptorDistance);
//            }
//        }
//    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::displayWord(int _wordId){
        auto &word = mWordDictionary[_wordId];

        cv::Mat complete;

        for(unsigned i = 0; i < word->frames.size(); i++){
            cv::Mat frame = mDataframes[word->frames[i]]->left.clone();
            auto &proj = word->projections[word->frames[i]];
            cv::Point p(proj[0], proj[1]);
            cv::circle(frame, p, 4, cv::Scalar(0,0,255),4);

            cv::putText(frame, "FRAME_"+std::to_string(word->frames[i]), cv::Point(50,50),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,255,0));
            if(complete.rows == 0)
                complete = frame.clone();
            else
                cv::hconcat(complete, frame, complete);
        }
        cv::imshow("Word_"+std::to_string(word->id), complete);
        cv::waitKey();
        cv::destroyWindow("Word_"+std::to_string(word->id));

    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::displaySharedWords(int _frame1, int _frame2){
        cv::Mat frame1 = mDataframes[_frame1]->left.clone();
        cv::Mat frame2 = mDataframes[_frame2]->left.clone();
        cv::Mat joinedFrames;
        cv::hconcat(frame1, frame2, joinedFrames);

        for(auto &word1:mDataframes[_frame1]->wordsReference){
            for(auto &word2:mDataframes[_frame2]->wordsReference){
                if(word1->id == word2->id){
                    auto &proj1 = word1->projections[_frame1];
                    auto &proj2 = word2->projections[_frame2];
                    cv::Point p1(proj1[0], proj1[1]);
                    cv::Point p2(proj2[0]+frame1.cols, proj2[1]);
                    cv::putText(joinedFrames, std::to_string(word1->id), p1,cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
                    cv::putText(joinedFrames, std::to_string(word2->id), p2,cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
                    cv::circle(joinedFrames, p1, 3, cv::Scalar(0,0,255),2);
                    cv::circle(joinedFrames,p2 , 3, cv::Scalar(0,0,255),2);
                    cv::line(joinedFrames, p1,p2,cv::Scalar(0,0,255), 1 );
                }
            }
        }
        cv::namedWindow(std::to_string(_frame1) + "_VS_"+ std::to_string(_frame2), CV_WINDOW_FREERATIO);
        cv::imshow(std::to_string(_frame1) + "_VS_"+ std::to_string(_frame2), joinedFrames);
        cv::waitKey();
        cv::destroyWindow(std::to_string(_frame1) + "_VS_"+ std::to_string(_frame2));

    }

}
