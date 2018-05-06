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

WrapperDarknet::WrapperDarknet(std::string mModelFile, std::string mWeightsFile) {
    char *wStr1= new char[mModelFile.size() + 1];
    char *wStr2= new char[mWeightsFile.size() + 1];

    std::copy(mModelFile.begin(), mModelFile.end(), wStr1);
    std::copy(mWeightsFile.begin(), mWeightsFile.end(), wStr2);

    wStr1[mModelFile.size()] = '\0';
    wStr2[mWeightsFile.size()] = '\0';

    cuda_set_device(0);

    mNet = *parse_network_cfg(wStr1);
    load_weights(&mNet, wStr2);


    set_batch_network(&mNet, 1);

    delete[] wStr1;
    delete[] wStr2;

    layer l = mNet.layers[mNet.n-1];

    mBoxes = (detection*) calloc(l.w*l.h*l.n, sizeof(detection));
    mProbs = (float**)calloc(l.w*l.h*l.n, sizeof(float *));
    for(int j = 0; j < l.w*l.h*l.n; ++j) mProbs[j] = (float *) calloc(l.classes + 1, sizeof(float *));
    mMasks = 0;
    if (l.coords > 4){
        mMasks = (float**) calloc(l.w*l.h*l.n, sizeof(float*));
        for(int j = 0; j < l.w*l.h*l.n; ++j) mMasks[j] = (float*) calloc(l.coords-4, sizeof(float *));
    }
}

std::vector<std::vector<float> > WrapperDarknet::detect(const cv::Mat &img) {
    srand(2222222);

    IplImage* iplImg = new IplImage(img);

    unsigned char *data = (unsigned char *)iplImg->imageData;
    int h = iplImg->height;
    int w = iplImg->width;
    int c = iplImg->nChannels;
    int step = iplImg->widthStep;
    image im = make_image(w, h, c);
    int i, j, k;

    #pragma omp parallel for
    for(i = 0; i < h; ++i){
        #pragma omp parallel for
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                im.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }


    image sized = letterbox_image(im, mNet.w, mNet.h);
    layer l = mNet.layers[mNet.n-1];



    float *X = sized.data;
    network_predict(&mNet, X);
    float thresh = 0.24;
    float hier_thresh = 0.5;
    int nboxes;
    mBoxes = get_network_boxes(&mNet, im.w, im.h, thresh, hier_thresh, 0, 1, &nboxes);
    float nms=0.3f;
    if (nms > 0) do_nms_obj(mBoxes, nboxes, l.classes, nms);

    std::vector<std::vector<float> > result;
    int num = l.w*l.h*l.n;
    for(int i = 0; i < num; ++i){
        std::vector<float> imgRes = {0};
        // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].

        int classi = max_index(mProbs[i], l.classes);
        float prob = mProbs[i][classi];
        if(prob > thresh){
            imgRes.push_back(classi);
            imgRes.push_back(prob);

            detection b = mBoxes[i];
            float left  = (b.bbox.x-b.bbox.w/2.);
            float top   = (b.bbox.y-b.bbox.h/2.);
            float right = (b.bbox.x+b.bbox.w/2.);
            float bot   = (b.bbox.y+b.bbox.h/2.);

            imgRes.push_back(left);
            imgRes.push_back(top);
            imgRes.push_back(right);
            imgRes.push_back(bot);
            result.push_back(imgRes);
        }
    }

    //free_image(im);
    free_image(sized);

    return result;
}
