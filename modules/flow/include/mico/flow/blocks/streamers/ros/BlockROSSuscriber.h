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
    template <typename ROSMessageType_ >
		struct BlockPolicy {
			BlockPolicy(std::string _blockName , std::vector<std::string> _tag ,std::vector< std::function < std::any(const typename ROSMessageType_::ConstPtr &)> > _fun){
				BlockName_ = _blockName;
				Tags_ = _tag;
				if (_tag.size() == _fun.size()){
					for (unsigned int idx = 0 ; idx < _tag.size() ; idx++)
						addToMap(_tag[idx] , _fun[idx]);
				}
			}

			std::vector<std::string> parameters() {
				return Tags_;
			}

			bool addToMap(std::string _tag, std::function < std::any(const typename ROSMessageType_::ConstPtr &)> _fun){
				callback_.insert(std::make_pair(_tag , _fun));
			}

            public:
                std::string BlockName_;
    		    std::vector<std::string> Tags_;
                ROSMessageType_ messageType_;
    		    std::map< std::string , std::function < std::any(const typename ROSMessageType_::ConstPtr &) > > callback_ ; // [tags , callbacks]
		};

	template<auto * _Policy , typename messageType >
    class BlockROSSuscriber : public Block{
    public:
		BlockROSSuscriber(){
            for (auto tag : _Policy->Tags_)
		        opipes_[tag] = new OutPipe(tag);
			}
		
        static std::string name() {return _Policy->BlockName_;}

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
			#ifdef MICO_USE_ROS
            	subROS_ = nh_.subscribe<messageType>(_params["topic"], 1 , &BlockROSSuscriber::subsCallback, this);
			#endif
	    	return true;
	    }

        std::vector<std::string> parameters() override {return {"topic"};} 

    private:
        void subsCallback(const typename messageType::ConstPtr &_msg){
		for(auto it = _Policy->callback_.begin() ; it != _Policy->callback_.end() ; it++ ){	
        	if(opipes_[it->first]->registrations() !=0 ){
               	opipes_[it->first]->flush(it->second(_msg));
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
