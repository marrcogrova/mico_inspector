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

        cuda_set_device(0);

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

        auto t0 = std::chrono::high_resolution_clock::now();
        // Prepare image
        cv::Mat bgr;
        cv::cvtColor(_img, bgr, CV_RGB2BGR);
        IplImage *iplImg = new IplImage(bgr);
        auto t0a = std::chrono::high_resolution_clock::now();

        // Create container
        int h = iplImg->height;
        int w = iplImg->width;
        int c = iplImg->nChannels;
        image im = make_image(w, h, c);
        auto t0b = std::chrono::high_resolution_clock::now();

        // Fill image with ipl data
        unsigned char *data = (unsigned char *)iplImg->imageData;
        int step = iplImg->widthStep;
        //#pragma omp parallel for
        for (int i = 0; i < h; ++i)
        {
            for (int k = 0; k < c; ++k)
            {
                for (int j = 0; j < w; ++j)
                {
                    im.data[k * w * h + i * w + j] = data[i * step + j * c + k] / 255.;
                }
            }
        }

        auto t0c = std::chrono::high_resolution_clock::now();
        image sized = letterbox_image(im, mNet->w, mNet->h);

        auto t0d = std::chrono::high_resolution_clock::now();
        // Get layer
        layer l = mNet->layers[mNet->n - 1];
        auto t1 = std::chrono::high_resolution_clock::now();
        float *X = sized.data;    // Copy datapointer
        network_predict(mNet, X); // Make prediction
        auto t2 = std::chrono::high_resolution_clock::now();
        int nboxes = 0;
        float thresh = 0.25;
        float hier_thresh = 0.4;
        float nms = 0.4;
        detection *dets = get_network_boxes(mNet, im.w, im.h, thresh, hier_thresh, 0, 1, &nboxes);
        if (nms)
            do_nms_sort(dets, nboxes, 1, nms);
        auto t3 = std::chrono::high_resolution_clock::now();

        int i, j;

        std::vector<std::vector<float>> result;
        for (i = 0; i < nboxes; ++i)
        {
            int classId = -1;
            float prob = 0;
            for (j = 0; j < 1; ++j)
            {
                if (dets[i].prob[j] > thresh)
                {
                    if (classId < 0)
                    {
                        classId = j;
                        prob = dets[i].prob[j];
                    }
                    // printf("%d: %.0f%%\n", classId, dets[i].prob[j]*100);
                }
            }

            if (classId >= 0)
            {
                int width = im.h * .006;

                box b = dets[i].bbox;
                //printf("%f %f %f %f\n", b.x, b.y, b.w, b.h);

                int left = (b.x - b.w / 2.) * im.w;
                int right = (b.x + b.w / 2.) * im.w;
                int top = (b.y - b.h / 2.) * im.h;
                int bot = (b.y + b.h / 2.) * im.h;

                if (left < 0)
                    left = 0;
                if (right > im.w - 1)
                    right = im.w - 1;
                if (top < 0)
                    top = 0;
                if (bot > im.h - 1)
                    bot = im.h - 1;

                result.push_back({classId, prob, left, top, right, bot});
            }
        }
        auto t4 = std::chrono::high_resolution_clock::now();

        //std::cout << "YOLO: imgPrep: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count() << std::endl;
        //std::cout << "YOLO: imgPrep: rgb2bgr: " << std::chrono::duration_cast<std::chrono::milliseconds>(t0a-t0).count() << std::endl;
        //std::cout << "YOLO: imgPrep: yolomake: " << std::chrono::duration_cast<std::chrono::milliseconds>(t0b-t0a).count() << std::endl;
        //std::cout << "YOLO: imgPrep: data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t0c-t0b).count() << std::endl;
        //std::cout << "YOLO: imgPrep: resize: " << std::chrono::duration_cast<std::chrono::milliseconds>(t0d-t0c).count() << std::endl;
        //std::cout << "YOLO: Net: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << std::endl;
        //std::cout << "YOLO: nms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3-t2).count() << std::endl;
        //std::cout << "YOLO: res: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4-t3).count() << std::endl;

        return result;
	#else
	return 	std::vector<std::vector<float>>();
	#endif
    }
}
