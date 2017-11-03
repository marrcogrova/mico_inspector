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


        class EdgeWritter {
            public:
                static bool getValueBetweenTwoFixedColors(float value, int &red, int &green, int &blue) {
                  int aR = 0;   int aG = 0; int aB=255;  // RGB for our 1st color (blue in this case).
                  int bR = 255; int bG = 0; int bB=0;    // RGB for our 2nd color (red in this case).

                  red   = (float)(bR - aR) * value + aR;      // Evaluated as -255*value + 255.
                  green = (float)(bG - aG) * value + aG;      // Evaluates as 0.
                  blue  = (float)(bB - aB) * value + aB;      // Evaluates as 255*value + 0.
                }
                // constructor - needs reference to graph we are coloring
                EdgeWritter( Graph& g ) : myGraph( g ) {}

                // functor that does the coloring
                template <class VertexOrEdge>
                void operator()(std::ostream& out, const VertexOrEdge& _e) const {
                    if(abs(source( _e, myGraph ) - target( _e, myGraph )) != 1){
                        float val = source( _e, myGraph ) / 200.0;
                        int r, g,b;
                        getValueBetweenTwoFixedColors(val, r, g, b);

                        std::stringstream str;
                        str.setf(std::ios_base::hex, std::ios::basefield);
                        str.setf(std::ios_base::uppercase);
                        str.fill('0');

                        str << std::setw(2) << (unsigned short)r;
                        str << std::setw(2) << (unsigned short)g;
                        str << std::setw(2) << (unsigned short)b;

                        out << "[color=\"#" <<str.str()<<"\"]";
                    }
                }
            private:
                Graph& myGraph;
        };
    };

}

#include "KeyframesToGraph.inl"

#endif
