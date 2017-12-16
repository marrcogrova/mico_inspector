//
//
//
//
//

#ifndef RGBD_MAP3D_WORD_H_
#define RGBD_MAP3D_WORD_H_

#include <vector>
#include <unordered_map>

namespace rgbd {
    struct Word{
        int id;
        std::vector<float> point;
        std::vector<int> frames;
        std::unordered_map<int, std::vector<float>> projections;
        std::unordered_map<int, int> idxInKf;
    };
}

#endif
