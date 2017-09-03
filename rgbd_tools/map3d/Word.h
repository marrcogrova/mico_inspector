//
//
//
//
//

#ifndef RGBD_MAP3D_WORD_H_
#define RGBD_MAP3D_WORD_H_

namespace rgbd {
    struct Word{
        int id;
        std::vector<float> point;
        std::vector<int> frames;
    };
}

#endif
