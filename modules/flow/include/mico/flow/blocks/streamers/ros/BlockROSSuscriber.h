//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#ifndef MICO_FLOW_BLOCKS_STREAMERS_ROS_ROSSUSCRIBER_H_
#define MICO_FLOW_BLOCKS_STREAMERS_ROS_ROSSUSCRIBER_H_

#include <mico/flow/Block.h>
#include <mico/flow/OutPipe.h>

#ifdef MICO_USE_ROS
	#include <ros/ros.h>
#endif

namespace mico{
	template<typename _Policy  >
    class BlockROSSuscriber : public Block{
    public:
		BlockROSSuscriber(){
            for (auto tag : _Policy::output_)
		        opipes_[tag] = new OutPipe(tag);
			}
		
        static std::string name() { 
			return _Policy::blockName_;
		}

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
			#ifdef MICO_USE_ROS
            	subROS_ = nh_.subscribe<typename _Policy::RosType_>(_params["topic"], 1 , &BlockROSSuscriber::subsCallback, this);
			#endif
	    	return true;
	    }

        std::vector<std::string> parameters() override {return {"topic"};} 

    private:
        void subsCallback(const typename _Policy::RosType_::ConstPtr &_msg){
			for (auto tag : _Policy::output_){
				if(opipes_[tag]->registrations() !=0 ){
               		opipes_[tag]->flush(_Policy::conversion_(tag , _msg));
				}
			}
        }

    private:
		#ifdef MICO_USE_ROS
			ros::NodeHandle nh_;
			ros::Subscriber subROS_;
		#endif
    };

}


#endif
