
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

    template<typename Block_>
    MicoFlowBlock<Block_>::MicoFlowBlock() {
        micoBlock_ = new Block_();
    }

    template<typename Block_>
    MicoFlowBlock<Block_>::~MicoFlowBlock(){
        delete micoBlock_;
    }

    template<typename Block_>
    void MicoFlowBlock<Block_>::inputConnectionDeleted(Connection const&_conn) {
        // Unregister element in policy
        auto tag = micoBlock_->getPolicy()->inputTags()[_conn.getPortIndex(PortType::In)];
        if(connectedPipes_[tag] != nullptr){
            connectedPipes_[tag]->unregisterPolicy(micoBlock_->getPolicy(), tag);
        }
    }

    template<typename Block_>
    unsigned int MicoFlowBlock<Block_>::nPorts(PortType portType) const {
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

    template<typename Block_>
    NodeDataType MicoFlowBlock<Block_>::dataType(PortType portType, PortIndex index) const {
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

    template<typename Block_>
    std::shared_ptr<NodeData> MicoFlowBlock<Block_>::outData(PortIndex index) {
        auto tag = micoBlock_->outputTags()[index];
        auto stream = micoBlock_->getStreams()[tag];
        std::shared_ptr<StreamerPipeInfo> ptr(new StreamerPipeInfo(stream, tag));  // 666 TODO
        return ptr;
    }


    template<typename Block_>
    void MicoFlowBlock<Block_>::setInData(std::shared_ptr<NodeData> data, PortIndex port) {
        // 666 Connections do not transfer data but streamers information to connect to internal block.
        if(data){
            auto pipeInfo = std::dynamic_pointer_cast<StreamerPipeInfo>(data)->info();
            if(pipeInfo.streamerRef_ != nullptr){
                micoBlock_->connect(pipeInfo.streamerRef_, {pipeInfo.pipeName_});
                connectedPipes_[pipeInfo.pipeName_] = pipeInfo.streamerRef_;
            }
        }
    }

}