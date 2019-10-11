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

#ifndef MICO_FLOW_BLOCKS_STREAMERS_ROSSUSCRIBER_H_
#define MICO_FLOW_BLOCKS_STREAMERS_ROSSUSCRIBER_H_

#include <mico/flow/Block.h>
#include <ros/ros.h>

namespace mico{

	template<typename ROSMessageType > // add callback to template
    class BlockROSSuscriber : public Block{
    public:
        static std::string name() {return "ROS Suscriber";}

		BlockROSSuscriber(){ opipes_["ROS topic"] = new OutPipe("ROS topic"); }
        
		// flush is used to?

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
			Topic_ = "/dummy_topic/topic";
		};
        std::vector<std::string> parameters() override;
        
    protected:
        virtual void loopCallback() override;

    private:
		// mTopic is a std::string with topic name
		// must send callback using template	
		ros::NodeHandle nh_;
		std::string Topic_;
		// ros::Subscriber subROS_ = nh_.subscribe<ROSMessageType>(Topic_, 1, [&](const ROSMessageType::ConstPtr &_msg){ 
        		// //_msg do things
				// std::cout << "suscribed node data" << std::endl;
    	// });
       
    };

}


#endif