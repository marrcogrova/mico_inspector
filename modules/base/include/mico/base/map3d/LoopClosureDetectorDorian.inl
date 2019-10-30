//---------------------------------------------------------------------------------------------------------------------
//  mico
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

namespace mico {
    //---------------------------------------------------------------------------------------------------------------------
    template <DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    bool LoopClosureDetectorDorian<DebugLevel_, OutInterface_>::init(cjson::Json &_config){
        // Set loop detector parameters
        typename BriefLoopDetector::Parameters params(640, 480);    // 666 image params
        // We are going to change these values individually:
        params.use_nss = true; // use normalized similarity score instead of raw score
        params.alpha = 0.3; // nss threshold
        params.k = 1; // a loop must be consistent with 1 previous matches
        params.geom_check = DLoopDetector::GEOM_DI; // use direct index for geometrical checking
        // params.geom_check = DLoopDetector::GEOM_FLANN;
        // params.geom_check = DLoopDetector::GEOM_NONE;
        // params.geom_check = DLoopDetector::GEOM_EXHAUSTIVE;
        params.di_levels = 2; // use two direct index levels
        params.max_ransac_iterations=4000;
        params.min_Fpoints = 7;
        params.max_reprojection_error = 5.0;
        mLoopClosureVoc = new BriefVocabulary((std::string)_config["vocabulary"]);
        mLoopClosureDetector = new BriefLoopDetector(*mLoopClosureVoc, params);
        mLoopClosureDetector->allocate(500);

        mFeatureDetector = cv::ORB::create(1000); // 666 configurable!

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    LoopResult LoopClosureDetectorDorian<DebugLevel_, OutInterface_>::appendCluster(cv::Mat &_image, int _id){
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> kpts;
        mFeatureDetector->detectAndCompute(_image, cv::Mat(), kpts, descriptors);   // 666 configurable detector
        return appendCluster(kpts, descriptors, _id);
    }


    //---------------------------------------------------------------------------------------------------------------------
    template <DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    LoopResult LoopClosureDetectorDorian<DebugLevel_, OutInterface_>::appendCluster(const std::vector<cv::KeyPoint> &_kps, const cv::Mat &_descriptors, int _id){
        std::vector<FBrief::TDescriptor> adaptedDescriptors;
        for(int i = 0; i < _descriptors.rows; i++){    // 666 Do it better but unexpected compilation issue
            std::bitset<256> bitset;
            cv::Mat row = _descriptors.row(i).clone();
            memcpy(&bitset, row.data, 32);
            adaptedDescriptors.push_back(bitset);
        } 

        DLoopDetector::DetectionResult result;
        mLoopClosureDetector->detectLoop(   
                                            _kps, 
                                            adaptedDescriptors, 
                                            result
                                            );

        if (result.detection()){
            this->message("LoopClosure", "Loop found with image " +std::to_string(result.match), mico::LogManager::cTextGreen);
        }
        else {
            switch (result.status) {
            case DLoopDetector::CLOSE_MATCHES_ONLY:
                this->message("LoopClosure", "All the images in the database are very recent", mico::LogManager::cTextYellow);
                break;
            case DLoopDetector::NO_DB_RESULTS:
                this->message("LoopClosure", "There are no matches against the database (few features in the image?)", mico::LogManager::cTextYellow);
                break;
            case DLoopDetector::LOW_NSS_FACTOR:
                this->message("LoopClosure", "Little overlap between this image and the previous one", mico::LogManager::cTextYellow);
                break;
            case DLoopDetector::LOW_SCORES:
                this->message("LoopClosure", "No match reaches the score threshold ", mico::LogManager::cTextYellow);
                break;
            case DLoopDetector::NO_GROUPS:
                this->message("LoopClosure",  "Not enough close matches to create groups - Best candidate: "+std::to_string( result.match), mico::LogManager::cTextYellow);
                break;
            case DLoopDetector::NO_TEMPORAL_CONSISTENCY:
                this->message("LoopClosure", "No temporal consistency - Best candidate: "+std::to_string( result.match) , mico::LogManager::cTextYellow);
                break;
            case DLoopDetector::NO_GEOMETRICAL_CONSISTENCY:
                this->message("LoopClosure", "No geometrical consistency. Best candidate: "+std::to_string( result.match), mico::LogManager::cTextYellow);
                break;
            default:
                break;
            }
        }

        mico::LoopResult finalResult;
        finalResult.found =  result.detection();
        finalResult.matchId =  result.match;
        return finalResult;
    }

};  // namespace mico 
