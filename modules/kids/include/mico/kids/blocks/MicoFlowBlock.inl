
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

    template<typename Block_, bool HasAutoLoop_>
    MicoFlowBlock<Block_,HasAutoLoop_>::MicoFlowBlock() {
        micoBlock_ = new Block_();

        if(micoBlock_->parameters().size() > 0){
            configsLayout_ = new QVBoxLayout();
            configBox_ = new QGroupBox("Configuration");
            configBox_->setLayout(configsLayout_);
            for(auto &param: micoBlock_->parameters()){
                configLabels_.push_back(new QLineEdit(param.c_str()));
                configsLayout_->addWidget(configLabels_.back());
            }
            configButton_ = new QPushButton("Configure");
            configsLayout_->addWidget(configButton_);
            connect(configButton_, &QPushButton::clicked, this, [this]() {
                std::unordered_map<std::string, std::string> params;
                int counter = 0; 
                for(auto &param: micoBlock_->parameters()){
                    params[param] =  configLabels_[counter]->text().toStdString();
                    counter++;
                }
                if(micoBlock_->configure(params)){
                    std::cout << "Configured block" << std::endl;
                }else{
                    std::cout << "Error configuring block" << std::endl;
                }
            });

            if(HasAutoLoop_){
                streamActionButton_ = new QCheckBox("Run");
                configsLayout_->addWidget(streamActionButton_);
                connect(    streamActionButton_, &QCheckBox::toggled,
                            [=](bool checked) { 
                                    if(checked){
                                        micoBlock_->start();
                                    }else{
                                        micoBlock_->stop();
                                    }
                                });
            }
        }
    }

    template<typename Block_, bool HasAutoLoop_>
    MicoFlowBlock<Block_,HasAutoLoop_>::~MicoFlowBlock(){
        delete micoBlock_;
    }


    template<typename Block_, bool HasAutoLoop_>
    QJsonObject MicoFlowBlock<Block_,HasAutoLoop_>::save() const{
        QJsonObject modelJson = NodeDataModel::save();

        unsigned counter = 0;
        for(auto &param: micoBlock_->parameters()){
            modelJson[param.c_str()] =  configLabels_[counter]->text();
            counter++;
        }

        return modelJson;
    }

    template<typename Block_, bool HasAutoLoop_>
    void MicoFlowBlock<Block_,HasAutoLoop_>::restore(QJsonObject const &p) {
        
        unsigned counter = 0;
        for(auto &param: micoBlock_->parameters()){
            QJsonValue v = p[param.c_str()];
            if (!v.isUndefined()) {
                QString strNum = v.toString();

                configLabels_[counter]->setText(strNum);
            }
            counter++;
        }
    }

    template<typename Block_, bool HasAutoLoop_>
    void MicoFlowBlock<Block_,HasAutoLoop_>::inputConnectionDeleted(Connection const&_conn) {
        // Unregister element in policy
        /*auto tag = micoBlock_->getPolicy()->inputTags()[_conn.getPortIndex(PortType::In)];
        if(connectedPipes_[tag] != nullptr){
            connectedPipes_[tag]->unregisterPolicy(micoBlock_->getPolicy());
        }*/
        assert(false); //666 not yet
    }

    template<typename Block_, bool HasAutoLoop_>
    unsigned int MicoFlowBlock<Block_,HasAutoLoop_>::nPorts(PortType portType) const {
        unsigned int result = 0;

        switch (portType) {
        case PortType::In:
            result = micoBlock_->nInputs();
            break;
        case PortType::Out:
            result = micoBlock_->nOutputs();
        default:
            break;
        }

        return result;
    }

    template<typename Block_, bool HasAutoLoop_>
    NodeDataType MicoFlowBlock<Block_,HasAutoLoop_>::dataType(PortType portType, PortIndex index) const {
        std::vector<std::string> tags;
        if(portType == PortType::In){
            tags = micoBlock_->inputTags();
        }else{
            tags = micoBlock_->outputTags();
        }

        assert(index < tags.size());
        
        auto iter = tags.begin() + index;
        std::string tag = *iter;
        
        return StreamerPipeInfo(nullptr, tag).type();
    }

    template<typename Block_, bool HasAutoLoop_>
    std::shared_ptr<NodeData> MicoFlowBlock<Block_,HasAutoLoop_>::outData(PortIndex index) {
        auto tag = micoBlock_->outputTags()[index];
        std::shared_ptr<StreamerPipeInfo> ptr(new StreamerPipeInfo(micoBlock_, tag));  // 666 TODO
                // std::cout << "pipeinfo " << tag << ", "  << micoBlock_ <<std::endl;
        return ptr;
    }


    template<typename Block_, bool HasAutoLoop_>
    void MicoFlowBlock<Block_,HasAutoLoop_>::setInData(std::shared_ptr<NodeData> data, PortIndex port) {
        // 666 Connections do not transfer data but streamers information to connect to internal block.
        if(data){
            auto pipeInfo = std::dynamic_pointer_cast<StreamerPipeInfo>(data)->info();
            if(pipeInfo.otherBlock_ != nullptr){
                pipeInfo.otherBlock_->connect(pipeInfo.pipeName_, *micoBlock_);
                //connectedPipes_[pipeInfo.pipeName_] = pipeInfo.otherBlock_;
            }
        }
    }

}