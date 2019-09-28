
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
    MicoFlowBlock<Block_>::MicoFlowBlock() : label_(new QLabel("Resulting Text"))
    {
        label_->setMargin(3);
        micoBlock_ = new Block_();
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
            assert(false);  // 666 Unxpected shit from node library
            break;
        }

        return result;
    }

    template<typename Block_>
    NodeDataType MicoFlowBlock<Block_>::dataType(PortType, PortIndex) const {
        return StreamerPipeInfo().type();
    }

    template<typename Block_>
    std::shared_ptr<NodeData> MicoFlowBlock<Block_>::outData(PortIndex) {
        std::shared_ptr<NodeData> ptr;  // 666 TODO
        return ptr;
    }


    template<typename Block_>
    void MicoFlowBlock<Block_>::setInData(std::shared_ptr<NodeData> data, PortIndex port) {
        // 666 Connections do not transfer data but streamers information to connect to internal block.
        auto pipeInfo = std::dynamic_pointer_cast<StreamerPipeInfo>(data)->info();
        micoBlock_->connect(pipeInfo.streamerRef_, {pipeInfo.pipeName_});
    }

}