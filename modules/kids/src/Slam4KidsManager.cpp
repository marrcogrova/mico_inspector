
//---------------------------------------------------------------------------------------------------------------------
//  mico
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


#include <mico/kids/Slam4KidsManager.h>

#include <nodes/NodeData>
#include <nodes/FlowScene>
#include <nodes/FlowView>

#include <QtWidgets/QApplication>

#include <mico/kids/blocks/MicoFlowBlock.h>
#include <mico/kids/blocks/MicoFlowStreamer.h>

#include <mico/flow/blocks/BlockImageVisualizer.h>
#include <mico/flow/blocks/BlockOdometryRGBD.h>
#include <mico/flow/blocks/BlockPointCloudVisualizer.h>
#include <mico/flow/blocks/BlockTrayectoryVisualizer.h>
#include <mico/flow/blocks/BlockDatabase.h>

#ifdef foreach  // To be able to use Qt and RealSense Device
  #undef foreach
#endif
#include <mico/flow/streamers/StreamRealSense.h>
#include <mico/flow/streamers/StreamPose.h>
#include <mico/flow/streamers/StreamDataset.h>

using QtNodes::DataModelRegistry;
using QtNodes::FlowView;
using QtNodes::FlowScene;

namespace mico{

    std::shared_ptr<DataModelRegistry> registerDataModels() {
        auto ret = std::make_shared<DataModelRegistry>();

        // Only streamers modules
        ret->registerModel<MicoFlowStreamer<StreamRealSense>>();
        ret->registerModel<MicoFlowStreamer<StreamPose>>();
        ret->registerModel<MicoFlowStreamer<StreamDataset, 3>>();

        // Processing and output modules
        ret->registerModel<MicoFlowBlock<BlockOdometryRGBD, 1>>();
        ret->registerModel<MicoFlowBlock<BlockImageVisualizer>>();
        ret->registerModel<MicoFlowBlock<BlockPointCloudVisualizer>>();
        ret->registerModel<MicoFlowBlock<BlockTrayectoryVisualizer>>();
        ret->registerModel<MicoFlowBlock<BlockDatabase, 1>>();

        return ret;
    }

    int Slam4KidsManager::init(int _argc, char** _argv){
        QApplication app(_argc, _argv);

        FlowScene scene(registerDataModels());

        FlowView view(&scene);

        view.setWindowTitle("Node-based flow editor");
        view.resize(800, 600);
        view.show();
        return app.exec();
    }


}