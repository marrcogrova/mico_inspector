
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

#ifndef MICO_KIDS_BLOCKS_MICOFLOWBLOCK_H_
#define MICO_KIDS_BLOCKS_MICOFLOWBLOCK_H_

#include <QtCore/QObject>
#include <QtWidgets/QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>

#include <mico/kids/data_types/StreamerPipeInfo.hpp>
#include <mico/flow/streamers/streamers.h>
#include <mico/flow/policies/policies.h>

#include <nodes/NodeDataModel>
#include <nodes/Connection>

#include <iostream>

using QtNodes::NodeData;
using QtNodes::NodeDataModel;
using QtNodes::PortIndex;
using QtNodes::PortType;
using QtNodes::Connection;

namespace mico{

    // Forward declaration
    class Ostream;

    template<typename Block_, int NConfigs_ = 0>
    class MicoFlowBlock : public NodeDataModel {
        
    public:
        MicoFlowBlock();

        virtual ~MicoFlowBlock();

    public:
        QString caption() const override { return Block_::name().c_str(); }

        bool captionVisible() const override { return true; }

        static QString Name() { return Block_::name().c_str(); }

        QString name() const override { return Block_::name().c_str(); }
    
        virtual void inputConnectionDeleted(Connection const&) override;

    public:
        unsigned int nPorts(PortType portType) const override;

        NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

        std::shared_ptr<NodeData> outData(PortIndex port) override;

        void setInData(std::shared_ptr<NodeData> data, PortIndex port) override;

        QWidget * embeddedWidget() override { return configBox_; }

    private:
        Block_ *micoBlock_;
        std::unordered_map<std::string, Ostream*> connectedPipes_;
        std::vector<QLineEdit*> configLabels_;
        QVBoxLayout *configsLayout_;
        QGroupBox *configBox_;

    };
}


#include <mico/kids/blocks/MicoFlowBlock.inl>

#endif