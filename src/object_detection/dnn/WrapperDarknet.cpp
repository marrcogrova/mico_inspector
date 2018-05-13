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

#include <rgbd_tools/object_detection/dnn/WrapperDarknet.h>
#include <chrono>

namespace rgbd{
    bool WrapperDarknet::init(std::string mModelFile, std::string mWeightsFile){
	#ifdef HAS_DARKNET
        char *wStr1 = new char[mModelFile.size() + 1];
        char *wStr2 = new char[mWeightsFile.size() + 1];

        std::copy(mModelFile.begin(), mModelFile.end(), wStr1);
        std::copy(mWeightsFile.begin(), mWeightsFile.end(), wStr2);

        wStr1[mModelFile.size()] = '\0';
        wStr2[mWeightsFile.size()] = '\0';

        cl_set_device(0);

        mNet = load_network(wStr1, wStr2, 0);
        set_batch_network(mNet, 1);
        srand(2222222);

        delete[] wStr1;
        delete[] wStr2;
        return mNet != nullptr;
	#else
	    return false;
	#endif
    }

    std::vector<std::vector<float>> WrapperDarknet::detect(const cv::Mat &_img) {
	#ifdef HAS_DARKNET
        if(mNet == nullptr){
            return std::vector<std::vector<float>>();
        }

        // Prepare image
        auto t0 = std::chrono::high_resolution_clock::now();
        cv::Mat bgr;
        cv::cvtColor(_img, bgr, CV_RGB2BGR);
        IplImage *iplImg = new IplImage(bgr);
        auto t1 = std::chrono::high_resolution_clock::now();
        
        // Create container
        int h = iplImg->height;
        int w = iplImg->width;
        int c = iplImg->nChannels;
        if(mLastW != w || mLastH != h || mLastC != c){
            mLastW = w; mLastH = h; mLastC = c;
            mImage = make_image(w, h, c);
        }
        auto t2 = std::chrono::high_resolution_clock::now();

        // Fill image with ipl data
        unsigned char *data = (unsigned char *)iplImg->imageData;
        int step = iplImg->widthStep;
        //#pragma omp parallel for
        for (int i = 0; i < h; ++i) {
            for (int k = 0; k < c; ++k) {
                for (int j = 0; j < w; ++j) {
                    mImage.data[k * w * h + i * w + j] = data[i * step + j * c + k] / 255.;
                }
            }
        }

        auto t3 = std::chrono::high_resolution_clock::now();
        image sized = letterbox_image(mImage, mNet->w, mNet->h);
        auto t4 = std::chrono::high_resolution_clock::now();
        // Get layer
        layer l = mNet->layers[mNet->n - 1];

        if(!mBoxes){
            mBoxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
        }

        if(!mProbs){
            mProbs= (float **)calloc(l.w*l.h*l.n, sizeof(float *));
            for(int j = 0; j < l.w*l.h*l.n; ++j) mProbs[j] = (float *)calloc(l.classes + 1, sizeof(float));
        }
        
        if (l.coords > 4){
            if(!mMasks){
                mMasks = (float **)calloc(l.w*l.h*l.n, sizeof(float*));
                for(int j = 0; j < l.w*l.h*l.n; ++j) mMasks[j] = (float*)calloc(l.coords-4, sizeof(float));
            }
        }
        auto t5 = std::chrono::high_resolution_clock::now();
        float *X = sized.data;
        network_predict(mNet, X);
        auto t6 = std::chrono::high_resolution_clock::now();
        get_region_boxes(l, mImage.w, mImage.h, mNet->w, mNet->h, thresh, mProbs, mBoxes, mMasks, 0, 0, hier_thresh, 1);
        auto t7 = std::chrono::high_resolution_clock::now();
        if (nms) do_nms_sort(mBoxes, mProbs, l.w*l.h*l.n, l.classes, nms);
        auto t8 = std::chrono::high_resolution_clock::now();
        int nboxes =l.w*l.h*l.n;

        std::vector<std::vector<float>> result;
        for (int i = 0; i < nboxes; ++i) {
            int classId = -1;
            float prob = 0;
            for (int j = 0; j < 1; ++j) {
                if (mProbs[i][j] > thresh) {
                    if (classId < 0) {
                        classId = j;
                        prob = mProbs[i][j];
                    }
                    // printf("%d: %.0f%%\n", classId, dets[i].prob[j]*100);
                }
            }

            if (classId >= 0) {
                int width = mImage.h * .006;

                box b = mBoxes[i];
                //printf("%f %f %f %f\n", b.x, b.y, b.w, b.h);

                int left = (b.x - b.w / 2.) * mImage.w;
                int right = (b.x + b.w / 2.) * mImage.w;
                int top = (b.y - b.h / 2.) * mImage.h;
                int bot = (b.y + b.h / 2.) * mImage.h;

                if (left < 0)
                    left = 0;
                if (right > mImage.w - 1)
                    right = mImage.w - 1;
                if (top < 0)
                    top = 0;
                if (bot > mImage.h - 1)
                    bot = mImage.h - 1;

                result.push_back({classId, prob, left, top, right, bot});
            }
        }
        auto t9 = std::chrono::high_resolution_clock::now();

        std::cout << "YOLO: image prep: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count() << std::endl; 
        std::cout << "YOLO: image copy: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3-t2).count() << std::endl; 
        std::cout << "YOLO: letter box: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4-t3).count() << std::endl; 
        std::cout << "YOLO: data init : " << std::chrono::duration_cast<std::chrono::milliseconds>(t5-t4).count() << std::endl; 
        std::cout << "YOLO: predict   : " << std::chrono::duration_cast<std::chrono::milliseconds>(t6-t5).count() << std::endl; 
        std::cout << "YOLO: get boxes : " << std::chrono::duration_cast<std::chrono::milliseconds>(t7-t6).count() << std::endl; 
        std::cout << "YOLO: nms       : " << std::chrono::duration_cast<std::chrono::milliseconds>(t8-t7).count() << std::endl; 
        std::cout << "YOLO: get result: " << std::chrono::duration_cast<std::chrono::milliseconds>(t8-t7).count() << std::endl; 

        return result;
	#else
	    return 	std::vector<std::vector<float>>();
	#endif
    }
}
