
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
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>

#ifdef foreach  // To be able to use Qt and RealSense Device
  #undef foreach
#endif

#include <mico/flow/flow.h>
#include <mico/kids/blocks/MicoFlowBlock.h>

#include <mico/kids/blocks/CastBlocks.h>

using QtNodes::DataModelRegistry;
using QtNodes::FlowView;
using QtNodes::FlowScene;

namespace mico{

    std::shared_ptr<DataModelRegistry> registerDataModels() {
        auto ret = std::make_shared<DataModelRegistry>();

        // Only streamers modules
        // ret->registerModel<MicoFlowStreamer<StreamRealSense>>           ("Streamers");
        // ret->registerModel<MicoFlowStreamer<StreamPose>>                ("Streamers");
        // ret->registerModel<MicoFlowStreamer<StreamDataset>>             ("Streamers");
        // ret->registerModel<MicoFlowStreamer<StreamPixhawk>>             ("Streamers");

        // // Processing and output modules
        // ret->registerModel<MicoFlowBlock<BlockImageVisualizer>>         ("Visualizers");
        // ret->registerModel<MicoFlowBlock<BlockPointCloudVisualizer>>    ("Visualizers");
        // ret->registerModel<MicoFlowBlock<BlockTrayectoryVisualizer>>    ("Visualizers");
        // ret->registerModel<MicoFlowBlock<BlockOdometryRGBD>>            ("Odometry");
        // ret->registerModel<MicoFlowBlock<BlockEKFIMU>>                  ("Estimators");
        // ret->registerModel<MicoFlowBlock<BlockDatabase>>                ("Mapping");

        // Casters
        ret->registerModel<MicoFlowBlock<BlockDataframeToPose>>         ("Cast");
        // ret->registerModel<MicoFlowBlock<BlockDataframeToCloud>>        ("Cast");
        // ret->registerModel<MicoFlowBlock<PoseDemux>>                    ("Cast");

        ret->registerModel<MicoFlowBlock<StreamDataset, true>>          ("Streamers");
        ret->registerModel<MicoFlowBlock<StreamRealSense, true>>        ("Streamers");

        ret->registerModel<MicoFlowBlock<BlockOdometryRGBD>>            ("Odometry");
        
        ret->registerModel<MicoFlowBlock<BlockImageVisualizer>>         ("Visualizers");
        ret->registerModel<MicoFlowBlock<BlockTrayectoryVisualizer>>    ("Visualizers");

        return ret;
    }

    int Slam4KidsManager::init(int _argc, char** _argv){
        QApplication app(_argc, _argv);

        QWidget mainWidget;
        auto menuBar    = new QMenuBar();
        auto saveAction = menuBar->addAction("Save..");
        auto loadAction = menuBar->addAction("Load..");

        QVBoxLayout *l = new QVBoxLayout(&mainWidget);
        l->addWidget(menuBar);
        auto scene = new FlowScene(registerDataModels(), &mainWidget);
        l->addWidget(new FlowView(scene));
        l->setContentsMargins(0, 0, 0, 0);
        l->setSpacing(0);

        QObject::connect(saveAction, &QAction::triggered, scene, &FlowScene::save);

        QObject::connect(loadAction, &QAction::triggered, scene, &FlowScene::load);

        mainWidget.setWindowTitle("Node-based flow editor");
        mainWidget.resize(800, 600);
        mainWidget.showNormal();

        return app.exec();
    }


}