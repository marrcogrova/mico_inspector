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



namespace mico {

	template<char const *BlockName_, char const *Tag_, typename ROSMessageType, typename ConversionCallback_>
    inline BlockROSSuscriber<BlockName_, Tag_, ROSMessageType, ConversionCallback_>::BlockROSSuscriber(){
		opipes_[Tag_] = new OutPipe(Tag_);

	}

	template<char const *BlockName_, char const *Tag_, typename ROSMessageType, typename ConversionCallback_>
    inline bool BlockROSSuscriber<BlockName_, Tag_, ROSMessageType, ConversionCallback_>::configure(std::unordered_map<std::string, std::string> _params){
		subROS_ = nh_.subscribe<ROSMessageType>(_params["topic"], 1 , ConversionCallback_);
		return true;
	}
	
	template<char const *BlockName_, char const *Tag_, typename ROSMessageType, typename ConversionCallback_>
    inline void BlockROSSuscriber<BlockName_, Tag_, ROSMessageType, ConversionCallback_>::subsCallback(const typename ROSMessageType::ConstPtr &_msg){    
		if(opipes_[Tag_]->registrations() !=0 )
			opipes_[Tag_]->flush(ConversionCallback_(_msg)); 
	}


} // namespace mico 
