////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

namespace rgbd{
    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    bool LoopClosureDetector<PointType_>::init(std::string _path) {
        mVocabulary.load(_path);
        return !mVocabulary.empty();
    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void LoopClosureDetector<PointType_>::update(std::shared_ptr<Keyframe<PointType_> > &_kf, Database<PointType_> &_database) {
        updateSimilarityMatrix(_kf, _database);
        checkLoopClosures(_database);
    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void LoopClosureDetector<PointType_>::updateSimilarityMatrix(std::shared_ptr<Keyframe<PointType_>> &_kf, Database<PointType_> &_database) {
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

    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void LoopClosureDetector<PointType_>::checkLoopClosures(Database<PointType_> &_database) {
        if(_database.numKeyframes() > mDistanceSearch){
            // Cover H matrix looking for loop.
            double min, max;
            cv::Point minLoc, maxLoc;
            cv::Mat lastRow = mCumulativeMatrix.row(_database.numKeyframes()-1);
            cv::minMaxLoc(lastRow, &min, &max, &minLoc, &maxLoc);
            cv::Point currLoc = maxLoc;currLoc.y = _database.numKeyframes()-1;
            cv::Mat loopsTraceBack = cv::Mat::zeros(_database.numKeyframes(), _database.numKeyframes(), CV_32F);
            std::vector<std::pair<int, int>> matches;
            while(currLoc.x > 0/*mCumulativeMatrix.rows*/ && currLoc.y>0/* < mCumulativeMatrix.cols*/){
                loopsTraceBack.at<float>(currLoc.y, currLoc.x) = 1;
                matches.push_back({currLoc.y, currLoc.x});
                float diagScore     = mCumulativeMatrix.at<float>(currLoc.y+1, currLoc.x+1);
                float downScore     = mCumulativeMatrix.at<float>(currLoc.y, currLoc.x+1);
                float rightScore    = mCumulativeMatrix.at<float>(currLoc.y+1, currLoc.x);

                if(diagScore> downScore && diagScore > rightScore){
                    currLoc.x--;//++;
                    currLoc.y--;//++;
                }else if(downScore > diagScore && downScore > rightScore){
                    currLoc.y--;//++;
                }else {
                    currLoc.x--;//++;
                }

                if(mCumulativeMatrix.at<float>(currLoc.y, currLoc.x) <= 0.01){
                    break;
                }
            }
            cv::Mat loopsTraceBackDisplay;
            cv::normalize(loopsTraceBack, loopsTraceBackDisplay, 0.0,1.0,CV_MINMAX);
            cv::imshow("loopTrace Matrix", loopsTraceBackDisplay);
            cv::waitKey(3);
            if(matches.size() > mBaSequenceSize){ // Loop closure detected! update kfs and world dictionary and perform the optimization.
                //for(unsigned i = 0; i < matches.size(); i++){
                //    Eigen::Matrix4f transformation; // Not used at all but needded in the interface
                //    auto kf1 = _database.keyframe(matches[i].first);
                //    auto kf2 = _database.keyframe(matches[i].second);
                //    transformationBetweenFeatures(kf1,kf2, transformation);
                //    // Update worldDictionary.
                //    _database.connectKeyframes(kf2->id, kf1->id);
                //}
                //
                //std::cout << "performing bundle adjustment!" << std::endl;
                //
                //if(!mAlreadyBaThread){
                //    mAlreadyBaThread = true;
                //    if(mBaThread.joinable())
                //        mBaThread.join();
                //
                //    mKeyframesBa = _database.keyframes();
                //    mBaThread = std::thread([&](){
                //        // Perform loop closure
                //        rgbd::BundleAdjuster<PointType_> ba;
                //        ba.keyframes(mKeyframesBa);
                //        ba.optimize();
                //        //  mKeyframes =ba.keyframes();
                //
                //        pcl::PointCloud<PointType_> map;
                //        for(auto &kf:mKeyframesBa){
                //            pcl::PointCloud<PointType_> cloud;
                //            pcl::transformPointCloudWithNormals(*kf->cloud, cloud, kf->pose);
                //            map += cloud;
                //        }
                //        pcl::VoxelGrid<PointType_> sor;
                //        sor.setInputCloud (mMap.makeShared());
                //        sor.setLeafSize (0.01f, 0.01f, 0.01f);
                //        sor.filter (map);
                //        mSafeMapCopy.lock();
                //        mMap = map;
                //        mSafeMapCopy.unlock();
                //        mAlreadyBaThread = false;
                //    });
                //}
            }
        }
    }
}
