//
//
//
//
//
//

#include <boost/graph/graphviz.hpp>
#include <fstream>
#include <graphviz/gvc.h>

namespace  rgbd {
    template<typename PointType_>
    void KeyframesToGraph<PointType_>::keyframes(std::vector<std::shared_ptr<Keyframe<PointType_>>> _kfs) {
        mKeyframes = _kfs;

        // Create edges.
        for(auto &kf: mKeyframes){
            for(auto iter = kf->multimatchesInliersKfs.begin(); iter != kf->multimatchesInliersKfs.end() ;iter++){
                int id1 = kf->id;
                int id2 = iter->first;
                boost::add_edge(id1, id2, mGraph);
            }
        }
        for(auto &kf: mKeyframes){
            mGraph[kf->id].x = kf->position[0]*100;
            mGraph[kf->id].y = kf->position[2]*100;
        }
    }

    template<typename PointType_>
    inline void KeyframesToGraph<PointType_>::save(std::string _path){
        std::ofstream outf(_path);
        boost::write_graphviz(outf, mGraph);
    }

    template<typename PointType_>
    inline void KeyframesToGraph<PointType_>::display() {
        std::ofstream outf("tempGraph.dot");
        outf << "strict ";
        boost::write_graphviz(outf, mGraph, DotWritter(mGraph), EdgeWritter(mGraph));
        outf.flush();
        outf.close();
        system("dot -n -Kfdp -Tpng tempGraph.dot -o tempOutGraph.png");
        cv::Mat graphImage;
        graphImage = cv::imread("tempOutGraph.png");
        if(graphImage.rows != 0){
            cv::namedWindow("graph", CV_WINDOW_FREERATIO);
            cv::imshow("graph", graphImage);
            cv::waitKey(3);
        }
    }

}
