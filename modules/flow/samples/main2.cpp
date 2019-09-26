

#include <mico/flow/streamers/streamers.h>
#include <mico/flow/streamers/StreamDataset.h>
#include <mico/flow/streamers/StreamRealSense.h>
#include <mico/flow/blocks/BlockOdometryRGBD.h>
#include <mico/flow/blocks/BlockImageVisualizer.h>
#include <mico/flow/blocks/BlockTrayectoryVisualizer.h>
#include <mico/flow/policies/policies.h>
#include <X11/Xlib.h>

#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

#include <gperftools/profiler.h>
#include <gperftools/heap-profiler.h>
#include <gperftools/heap-checker.h>

using namespace mico;

int main(){
    XInitThreads();
    // ProfilerStart("profiler.log");
    
    // Stream definition
    // OstreamDataset stream;
    // stream.configure({  {"left","/home/bardo91/programming/rgbd_dataset_freiburg1_room/rgb/left_%d.png"},
    //                     {"depth","/home/bardo91/programming/rgbd_dataset_freiburg1_room/depth/depth_%d.png"},
                        // {"calibFile","/home/bardo91/programming/rgbd_dataset_freiburg1_room/CalibrationFile_fr1.xml"}});

    OstreamRealsense stream;

    // OdometryBlock
    BlockOdometryRGBD blockOdom;
    blockOdom.connect(&stream, {"rgb", "depth", "cloud"});

    BlockImageVisualizer blockVis;
    blockVis.connect(&stream, {"rgb"});

    BlockTrayectoryVisualizer blockTraj;
    blockTraj.connect(blockOdom.getStreams()["pose"], {"pose"});

    // Start streaming
    stream.start();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    

    stream.stop();

    // ProfilerStop();
    
    
}