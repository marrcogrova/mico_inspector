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

namespace rgbd{
    template<typename PointType_>
    void Database<PointType_>::addDataframe(std::shared_ptr<DataFrame<PointType_> > &_kf) {
        std::vector<cv::Mat> descriptors;
        for(unsigned  r = 0; r < _kf->featureDescriptors.rows; r++){
            descriptors.push_back(_kf->featureDescriptors.row(r));
        }
        mVocabulary.transform(descriptors, _kf->signature,_kf->featVec,4);

        mDataframes.push_back(_kf);
        if(mClusters.size() == 0){
            std::shared_ptr<ClusterFrames<PointType_>> cluster = std::shared_ptr<ClusterFrames<PointType_>>(new ClusterFrames<PointType_>);
            mClusters.push_back(cluster);
            mClusters[0]->frames.push_back(_kf);
        }else{
            // Compare Kfs
            double score = mVocabulary.score(_kf->signature, mClusters.back()->frames[0]->signature);
            std::cout << "Score between frame " << _kf->id << " and " << mClusters.back()->frames[0]->id << ": " << score << std::endl;
            if(score > 0.4){ // 666 CHECK PARAM!!
                mClusters.back()->frames.push_back(_kf);
            }else{
                std::shared_ptr<ClusterFrames<PointType_>> cluster = std::shared_ptr<ClusterFrames<PointType_>>(new ClusterFrames<PointType_>);
                mClusters.push_back(cluster);
                mClusters.back()->frames.push_back(_kf);
            }
        }
    }

    /*//---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::connectKeyframes(unsigned _id1, unsigned _id2, bool _has3D){
        auto &kf = mKeyframes[_id1];
        auto &prevKf = mKeyframes[_id2];
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
                if(!_has3D){
                    auto triangulatedPoint = triangulateFromProjections(prevWord->projections);
                    prevWord->point = { triangulatedPoint(0), triangulatedPoint(1), triangulatedPoint(2) };
                    if(mWordMap.size() < prevWord->id+1){
                        PointType_ p;
                        p.x = prevWord->point[0];
                        p.y = prevWord->point[1];
                        p.z = prevWord->point[2];
                        mWordMap.push_back(p);
                    }else{
                        PointType_ p;
                        p.x = prevWord->point[0];
                        p.y = prevWord->point[1];
                        p.z = prevWord->point[2];
                        mWordMap[prevWord->id] = p;
                    }
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
    }*/

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::reset() {
        mClusters.clear();
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

}
