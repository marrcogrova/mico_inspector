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

#include <string>
#include <unordered_map>

#include <mico/base/StereoCamera.h>
#include <mico/base/cjson/json.h>

#include <mico/base/object_detection/feature_based/FeatureObjectTracker.h>

int main(int _argc, char** _argv){
	cjson::Json mConfigFile;
	mico::StereoCamera *mCamera;

	if (_argc != 2) {
        std::cout << "Bad input arguments, please provide only the path of a json config file with the structure detailed in the documentation" << std::endl;
        return -1;
    }

    std::ifstream file(_argv[1]);
    if (!file.is_open()) {
        std::cout << "Error opening config file" << std::endl;
        return -1;
    }

    if (!mConfigFile.parse(file)) {
        std::cout << "Error parsing config file" << std::endl;
        return -1;
    }

	// Instantiate camera
	mCamera = mico::StereoCamera::create((std::string) mConfigFile["cameraType"]);

	// Init camera
	if (mCamera == nullptr || !mCamera->init(mConfigFile["deviceConfig"])) {
	    std::cout << "Failed initialization of the camera" << std::endl;
	    return false;
	}

    mico::FeatureObjectTracker tracker;
    if(!tracker.init(mConfigFile)){
        std::cout << "Failed object tracker initialization" << std::endl;
        return -1;
    }

	while(true){
        mCamera->grab();
        cv::Mat image, dummy;
        mCamera->rgb(image, dummy);
        cv::Mat position, orientation;
        if(tracker.update(image, position, orientation))
            tracker.drawCoordinate(position, orientation, image);

        tracker.drawCurrentWindow(image);
        cv::imshow("display", image);
        cv::waitKey();
	}
}
