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



namespace mico{

	template<typename ROSMessageType>
    inline BlockROSSuscriber<ROSMessageType>::BlockROSSuscriber(){
		opipes_["ROStopic"] = new OutPipe("ROStopic");

	}

	template<typename ROSMessageType>
    inline bool BlockROSSuscriber<ROSMessageType>::configure(std::unordered_map<std::string, std::string> _params){
		topic_ = "/dummy_topic/topic";

		init();

		return true;
	}
	
	template<typename ROSMessageType>
    inline void BlockROSSuscriber<ROSMessageType>::loopCallback(const typename ROSMessageType::ConstPtr &_msg){
		while(runLoop_){
            // convert ros_msg to std::mico
			Eigen::Vector3f data;
            
			if(opipes_["ROStopic"]->registrations() !=0 ){
				//memcpy(data , _msg , _msg->size() );
				// here convert _msg to data
				//
				
                if(data.size() != 0)
                    opipes_["ROStopic"]->flush(data); 
            }
                
        }
		return;
	}

	template <typename ROSMessageType>
	inline bool BlockROSSuscriber<ROSMessageType>::init(){

		subROS_ = nh_.subscribe<ROSMessageType>(topic_, 1 , &BlockROSSuscriber<ROSMessageType>::loopCallback);
		return true;
	}
	


} // namespace mico 
