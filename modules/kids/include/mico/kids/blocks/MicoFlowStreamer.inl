
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

    template<typename Streamer_>
    MicoFlowStreamer<Streamer_>::MicoFlowStreamer() : streamActionButton_(new QCheckBox("Run"))
    {
        micoStreamer_ = new Streamer_();

        streamerBox_ = new QGroupBox();
        auto streamerLayout = new QVBoxLayout();
        streamerLayout->addWidget(streamActionButton_);
        streamerBox_->setLayout(streamerLayout);
        if(micoStreamer_->parameters().size() > 0){
            configsLayout_ = new QVBoxLayout();
            configBox_ = new QGroupBox("Configuration");
            configBox_->setLayout(configsLayout_);
            for(auto &param: micoStreamer_->parameters()){
                configLabels_.push_back(new QLineEdit(param.c_str()));
                configsLayout_->addWidget(configLabels_.back());
            }
            streamerLayout->addWidget(configBox_);
        }


        connect(    streamActionButton_, &QCheckBox::toggled,
                    [=](bool checked) { 
                            if(checked){
                                micoStreamer_->start();
                            }else{
                                micoStreamer_->stop();
                            }
                        });
    }


    template<typename Streamer_>
    MicoFlowStreamer<Streamer_>::~MicoFlowStreamer(){
        delete micoStreamer_;
    }

    template<typename Streamer_>
    unsigned int MicoFlowStreamer<Streamer_>::nPorts(PortType portType) const {
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

    template<typename Streamer_>
    NodeDataType MicoFlowStreamer<Streamer_>::dataType(PortType portType, PortIndex index) const {
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

    template<typename Streamer_>
    std::shared_ptr<NodeData> MicoFlowStreamer<Streamer_>::outData(PortIndex index) {
        auto tag = micoStreamer_->outputTags()[index];
        std::shared_ptr<StreamerPipeInfo> ptr(new StreamerPipeInfo(micoStreamer_, tag));  // 666 TODO
        return ptr;
    }


    template<typename Streamer_>
    void MicoFlowStreamer<Streamer_>::setInData(std::shared_ptr<NodeData> data, PortIndex port) {
       // This is not expected to happen
       assert(false);
    }

}