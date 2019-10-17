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


#ifndef MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKQUEUER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_PROCESSORS_BLOCKQUEUER_H_

#include <mico/flow/Block.h>
#include <mico/base/map3d/OdometryRgbd.h>
#include <queue>


namespace mico{

    template<typename Trait_>
    class BlockQueuer: public Block{
    public:
        static std::string name() {return Trait_::Name_;}

        BlockQueuer(){
            opipes_[Trait_::Output_] = new OutPipe(Trait_::Output_);
            iPolicy_ = new Policy({Trait_::Input_});
            iPolicy_->setCallback({Trait_::Input_}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        typename Trait_::Type_ data = std::any_cast<typename Trait_::Type_>(_data[Trait_::Input_]);
                                        queue_.push(data);
                                        if(queue_.size() > size_){
                                            queue_.pop();
                                            opipes_[Trait_::Output_]->flush(queue_);
                                        }
                                        idle_ = true;
                                    }
                                }
            );
        }

        bool configure(std::unordered_map<std::string, std::string> _params) override{
            size_ = atoi(_params["queue_size"].c_str());
            return true;
        }

        std::vector<std::string> parameters() override{
            return {"queue_size"};
        }
    
    private:
        std::queue<typename Trait_::Type_> queue_;
        int size_ = 1;
        bool idle_ = true;
    };


    //-----------------------------------------------------------------------------------------------------------------
    struct QueuerTraitClusterframes{
        constexpr static const char * Name_ = "Queuer Clusterframes";
        constexpr static const char * Output_ = "v-clusterframes";
        constexpr static const char * Input_ = "clusterframes";
        typedef mico::ClusterFrames<pcl::PointXYZRGBNormal>::Ptr Type_;
    };

}

#endif