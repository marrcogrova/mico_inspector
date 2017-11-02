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
            double x;
            double y;
        };

        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, vertexInfo> Graph;
        Graph mGraph;

        class DotWritter {
            public:

            // constructor - needs reference to graph we are coloring
            DotWritter( Graph& g ) : myGraph( g ) {}

                // functor that does the coloring
                template <class Vertex>
                void operator()(std::ostream& out, const Vertex& _v) const {
                    // check if this is the edge we want to color red
                    out << "[width=0.2, height=0.2,pin=true,pos=\""<< myGraph[_v].x << ","<< myGraph[_v].y <<"!\"]";
                }
            private:
                Graph& myGraph;
        };
    };

}

#include "KeyframesToGraph.inl"

#endif
