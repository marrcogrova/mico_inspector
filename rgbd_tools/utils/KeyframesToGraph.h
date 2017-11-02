//
//
//
//
//
//

#ifndef RGBDTOOLS_UTILS_KEYFRAMESTOGRAPH_H_
#define RGBDTOOLS_UTILS_KEYFRAMESTOGRAPH_H_

#include<vector>
#include <string>

#include "../map3d/keyframe.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace rgbd{

    template<typename PointType_>
    class KeyframesToGraph{
    public:
        void keyframes(std::vector<std::shared_ptr<Keyframe<PointType_>>> _kfs);

        void save(std::string _path);

        void display();
    private:
        std::vector<std::shared_ptr<Keyframe<PointType_>>> mKeyframes;

        struct vertexInfo{
            int id;
            int color;
        };

        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, vertexInfo> Graph;
        Graph mGraph;

    };

}

#include "KeyframesToGraph.inl"

#endif
