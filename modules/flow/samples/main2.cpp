

#include <mico/flow/streamers/streamers.h>
#include <mico/flow/streamers/StreamDataset.h>
#include <mico/flow/blocks/BlockOdometryRGBD.h>
#include <mico/flow/blocks/BlockImageVisualizer.h>
#include <mico/flow/policies/policies.h>


#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

#include <gperftools/profiler.h>
#include <gperftools/heap-profiler.h>
#include <gperftools/heap-checker.h>

using namespace mico;

int main(){
    
    ProfilerStart("profiler.log");
    // OdometryBlock
    BlockOdometryRGBD blockOdom;
    PolicyAllRequired pol;
    blockOdom.setPolicy(&pol);
    
    // Vis block
    BlockImageVisualizer blockVis1;
    PolicyAllRequired pol2;
    blockVis1.setPolicy(&pol2);
    
    // Stream definition
    OstreamDataset stream;
    stream.configure({  {"left","/home/bardo91/programming/rgbd_dataset_freiburg1_room/rgb/left_%d.png"},
                        {"depth","/home/bardo91/programming/rgbd_dataset_freiburg1_room/depth/depth_%d.png"},
                        {"calibFile","/home/bardo91/programming/rgbd_dataset_freiburg1_room/CalibrationFile_fr1.xml"}});

    // Register blocks
    stream.registerPolicy(&pol, 0);
    stream.registerPolicy(&pol, 1);
    stream.registerPolicy(&pol, 2);

    stream.registerPolicy(&pol2, 0);

    // Start streaming
    stream.start();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    stream.stop();

    ProfilerStop();
    
    
}