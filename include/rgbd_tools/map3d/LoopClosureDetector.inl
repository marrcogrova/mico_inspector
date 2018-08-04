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


#include <rgbd_tools/map3d/utils3d.h>

namespace rgbd{
    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    bool LoopClosureDetector<PointType_>::init(std::string _path) {
        #ifdef USE_DBOW2
            mVocabulary.load(_path);
            //mVocabulary.setScoringType(DBoW2::L2_NORM);  //TODO: Change this scoring type
            return !mVocabulary.empty();
        #else
            return false;
        #endif
    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void LoopClosureDetector<PointType_>::update(Database<PointType_> &_database) {
        #ifdef USE_DBOW2
            //updateSimilarityMatrix(_kf, _database);
            checkLoopClosures(_database);
        #endif
    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void LoopClosureDetector<PointType_>::updateSimilarityMatrix(std::shared_ptr<DataFrame<PointType_>> &_kf, Database<PointType_> &_database) {
        #ifdef USE_DBOW2
            // Computing signature  666 not sure if should be here or in a common place!
            std::vector<cv::Mat> descriptors;
            for(unsigned  r = 0; r < _kf->featureDescriptors.rows; r++){
                descriptors.push_back(_kf->featureDescriptors.row(r));
            }
            mVocabulary.transform(descriptors, _kf->signature);

            // Start Smith-Waterman algorithm
            cv::Mat newMatrix(_database.numKeyframes(), _database.numKeyframes(),CV_32F);   // 666 look for a more efficient way of dealing with the similarity matrix. Now it is recomputed all the time!
            mSimilarityMatrix.copyTo(newMatrix(cv::Rect(0,0,mSimilarityMatrix.cols,mSimilarityMatrix.rows)));
            mSimilarityMatrix = newMatrix;

            // BUILD M matrix, i.e., similarity matrix.
            for(unsigned kfId = 0; kfId < _database.numKeyframes(); kfId++){
                double score = mVocabulary.score(_kf->signature, _database.keyframe(kfId)->signature);
                mSimilarityMatrix.at<float>(kfId, _kf->id) = score;
                mSimilarityMatrix.at<float>(_kf->id, kfId) = score;
            }
            cv::Mat similarityDisplay;
            cv::normalize(mSimilarityMatrix, similarityDisplay, 0.0,1.0,CV_MINMAX);
            cv::imshow("Similarity Matrix", similarityDisplay);
            cv::waitKey(3);

            //cv::Mat Mp = Mr.clone();
            cv::Mat Mp = mSimilarityMatrix.clone();

            if(_database.numKeyframes() > mDistanceSearch){
                // Build H matrix, cummulative matrix
                float penFactor = 0.05;
                mCumulativeMatrix = cv::Mat::zeros(_database.numKeyframes(), _database.numKeyframes(), CV_32F);
                for(unsigned j = mCumulativeMatrix.cols-1 - mDistanceSearch; j >= 1 ; j--){
                    for(unsigned i = mCumulativeMatrix.rows-1; i >= j + mDistanceSearch; i--){
                        float diagScore = mCumulativeMatrix.at<float>(i-1, j-1) + Mp.at<float>(i,j);
                        float upScore   = mCumulativeMatrix.at<float>(i-1, j) + Mp.at<float>(i,j) - penFactor;
                        float leftScore = mCumulativeMatrix.at<float>(i, j-1) + Mp.at<float>(i,j) - penFactor;

                        mCumulativeMatrix.at<float>(i,j) = std::max(std::max(diagScore, upScore), leftScore);
                    }
                }
                cv::Mat Hdisplay;
                cv::normalize(mCumulativeMatrix, Hdisplay, 0.0,1.0,CV_MINMAX);
                cv::imshow("H Matrix", Hdisplay);
                cv::waitKey(3);
            }
        #else
            std::cout << "RGBD_TOOLS has been built without DBoW2 support, loop closure is disabled" << std::endl;
        #endif
    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void LoopClosureDetector<PointType_>::checkLoopClosures(Database<PointType_> &_database) {
        std::cout << "Loop Closure Detection started " << std::endl;
        double score;
        std::unordered_map<int, std::shared_ptr<ClusterFrames<PointType_>>> Clusters=_database.clusters();
        auto lCluster=_database.rlastCluster();
        auto _kf=Clusters[lCluster->id]->frames[0];
        for(auto it=Clusters.begin(); it != Clusters.end() ; ++it){
            score=mVocabulary.score(_kf->signature,it->second->frames[0]->signature);
            _database.changeRelations(lCluster->id,it->first,score);
            if(it->first!=lCluster->id && (lCluster->id)-(it->first)>6 && score<0.3){
                std::cout << "-----------------------------LOOP CLOSURE DETECTED-----------------------------" << std::endl;
                std::cout << "Score between cluster " << lCluster->id <<  " and "  << it->first << " is: " << score << std::endl;
            }
        }
    }


    //---------------------------------------------------------------------------------------------------------------------
//    template<typename PointType_>
//    inline bool  LoopClosureDetector<PointType_>::transformationBetweenFeatures(std::shared_ptr<DataFrame<PointType_>> &_previousKf, std::shared_ptr<DataFrame<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
//        if(_currentKf->multimatchesInliersKfs.find(_previousKf->id) !=  _currentKf->multimatchesInliersKfs.end()){
//            // Match already computed
//            std::cout << "Match alread computed between frames: " <<_currentKf->id << " and " << _previousKf->id << std::endl;
//            return true;
//        }
//        std::vector<cv::DMatch> matches;
//        matchDescriptors(_currentKf->featureDescriptors, _previousKf->featureDescriptors, matches);

//        std::vector<int> inliers;
//        rgbd::ransacAlignment<PointType_>(_currentKf->featureCloud, _previousKf->featureCloud, matches,_transformation, inliers, 0.03, 3000);

//        if (inliers.size() >= 12) {
//            _currentKf->multimatchesInliersKfs[_previousKf->id];
//            _previousKf->multimatchesInliersKfs[_currentKf->id];
//            int j = 0;
//            for(int i = 0; i < inliers.size(); i++){
//                while(matches[j].queryIdx != inliers[i]){
//                    j++;
//                }
//                _currentKf->multimatchesInliersKfs[_previousKf->id].push_back(matches[j]);
//                _previousKf->multimatchesInliersKfs[_currentKf->id].push_back(cv::DMatch(matches[j].trainIdx, matches[j].queryIdx, matches[j].distance));

//            }
//            return true;
//        }else{

//            return false;
//        }
//    }

    //---------------------------------------------------------------------------------------------------------------------
//    template<typename PointType_>
//    inline bool LoopClosureDetector<PointType_>::matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers) {
//        std::vector<cv::DMatch> matches12, matches21;
//        cv::BFMatcher featureMatcher;
//        featureMatcher.match(_des1, _des2, matches12);
//        featureMatcher.match(_des2, _des1, matches21);

//        double max_dist = 0; double min_dist = 999999;
//        //-- Quick calculation of max and min distances between keypoints
//        for( int i = 0; i < _des1.rows; i++ ) {
//            double dist = matches12[i].distance;
//            if( dist < min_dist ) min_dist = dist;
//            if( dist > max_dist ) max_dist = dist;
//        }

//        // symmetry test.
//        for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
//            for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
//                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
//                    if(it12->distance <= min_dist*8){
//                        _inliers.push_back(*it12);
//                    }
//                    break;
//                }
//            }
//        }
//        return true;
//    }
}
