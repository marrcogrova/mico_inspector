////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <Eigen/Eigen>

#include <dlib/optimization.h>
#include <dlib/global_optimization.h>

namespace rgbd{
    template<typename PointType_>
    void Database<PointType_>::addKeyframe(std::shared_ptr<DataFrame<PointType_> > &_kf) {
        mKeyframes.push_back(_kf);
    }

    //---------------------------------------------------------------------------------------------------------------------
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
                int wordId = mWorldDictionary.size();
                mWorldDictionary[wordId] = std::shared_ptr<Word>(new Word);
                mWorldDictionary[wordId]->id        = wordId;
                if(_has3D){
                    mWorldDictionary[wordId]->point     = {transCloud.at(inlierIdxInCurrent).x, transCloud.at(inlierIdxInCurrent).y, transCloud.at(inlierIdxInCurrent).z};
                    PointType_ p;
                    p.x = mWorldDictionary[wordId]->point[0];
                    p.y = mWorldDictionary[wordId]->point[1];
                    p.z = mWorldDictionary[wordId]->point[2];
                    mWordMap.push_back(p);
                }
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

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Database<PointType_>::reset() {
        mKeyframes.clear();
        mWorldDictionary.clear();
        mWordMap.clear();
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
            auto kf = mKeyframes[id];
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
