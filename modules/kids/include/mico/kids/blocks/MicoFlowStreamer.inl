
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


#include <mico/kids/data_types/StreamerPipeInfo.hpp>

namespace mico{

    template<typename Streamer_, int NConfigs_ >
    MicoFlowStreamer<Streamer_, NConfigs_ >::MicoFlowStreamer() : streamActionButton_(new QCheckBox("Run")), configLabels_(NConfigs_)
    {
        streamerBox_ = new QGroupBox();
        auto streamerLayout = new QVBoxLayout();
        streamerLayout->addWidget(streamActionButton_);
        streamerBox_->setLayout(streamerLayout);
        if(NConfigs_){
            configsLayout_ = new QVBoxLayout();
            configBox_ = new QGroupBox("Configuration");
            configBox_->setLayout(configsLayout_);
            for(unsigned i = 0; i < NConfigs_; i++){
                configLabels_[i] = new QLineEdit(("Parameter_"+std::to_string(i)).c_str());
                configsLayout_->addWidget(configLabels_[i]);
            }
            streamerLayout->addWidget(configBox_);
        }

        micoStreamer_ = new Streamer_();

        connect(    streamActionButton_, &QCheckBox::toggled,
                    [=](bool checked) { 
                            if(checked){
                                micoStreamer_->start();
                            }else{
                                micoStreamer_->stop();
                            }
                        });
    }


    template<typename Streamer_, int NConfigs_ >
    MicoFlowStreamer<Streamer_, NConfigs_>::~MicoFlowStreamer(){
        delete micoStreamer_;
    }

    template<typename Streamer_, int NConfigs_ >
    unsigned int MicoFlowStreamer<Streamer_, NConfigs_>::nPorts(PortType portType) const {
        unsigned int result = 0;

        switch (portType) {
        case PortType::In:
            break;
        case PortType::Out:
            result = micoStreamer_->nOutputs();
        default:
            break;
        }

        return result;
    }

    template<typename Streamer_, int NConfigs_ >
    NodeDataType MicoFlowStreamer<Streamer_, NConfigs_>::dataType(PortType portType, PortIndex index) const {
        std::vector<std::string> tags;
        if(portType == PortType::In){
        }else{
            tags = micoStreamer_->outputTags();
        }

        assert(index < tags.size());
        
        auto iter = tags.begin() + index;
        std::string tag = *iter;
        
        return StreamerPipeInfo(nullptr, tag).type();
    }

    template<typename Streamer_, int NConfigs_ >
    std::shared_ptr<NodeData> MicoFlowStreamer<Streamer_, NConfigs_>::outData(PortIndex index) {
        auto tag = micoStreamer_->outputTags()[index];
        std::shared_ptr<StreamerPipeInfo> ptr(new StreamerPipeInfo(micoStreamer_, tag));  // 666 TODO
        return ptr;
    }


    template<typename Streamer_, int NConfigs_ >
    void MicoFlowStreamer<Streamer_, NConfigs_>::setInData(std::shared_ptr<NodeData> data, PortIndex port) {
       // This is not expected to happen
       assert(false);
    }

}