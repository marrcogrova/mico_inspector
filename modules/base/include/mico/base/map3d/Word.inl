//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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

namespace mico 
{   

    template<typename PointType_>
    inline Word<PointType_>::Word(int _wordId, std::vector<float> _point3D, cv::Mat _descriptor){
        id = _wordId;
        point = _point3D;
        descriptor = _descriptor;
    }

    template<typename PointType_>
    inline void Word<PointType_>::addDataframe(int _frameId){
        dfIds.push_back(_frameId);
    }

    template<typename PointType_>
    inline void Word<PointType_>::addObservation(   int _dfId, 
                                                    std::shared_ptr<Dataframe<PointType_>> _df,
                                                    int _idx,
                                                    std::vector<float> _projections){
        /// 666 Shouldn't we check if it already exists?
        dfIds.push_back(_dfId);
        projections[_dfId] = _projections;
        projectionsEnabled[_dfId] = true;
        idxInDf[_dfId] = _idx;
        dfMap[_dfId] = _df;
    }


    template<typename PointType_>
    inline void Word<PointType_>::mergeWord(std::shared_ptr<Word<PointType_>> _word){
        // Add df ids
        for(auto &newCf: _word->dfIds){
            if(std::find(dfIds.begin(), dfIds.end(), newCf) == dfIds.end()){
                dfIds.push_back(newCf);
            }
        }

        // Check new projections
        for(auto &proj: _word->projections){
            if(projections.find(proj.first)==projections.end()){    
                // Add new projection
                projections[proj.first] = proj.second;
                projectionsEnabled[proj.first] = true;  // TODO: ?? 
            }
        }

        // Check for new idx in dfIds
        for(auto &idx: _word->idxInDf){
            if(idxInDf.find(idx.first)==idxInDf.end()){    
                // Add new idx
                idxInDf[idx.first] = idx.second;
            }
        }

        // Add pointer of new dataframe
        for(auto &newDf: _word->dfMap){
            if(dfMap.find(newDf.first)==dfMap.end()){    
                // Add new dataframe pointer
                dfMap[newDf.first] = newDf.second;

                // Update covisibility of dataframe
                for(auto &currentDf: dfMap){
                    newDf.second->appendCovisibility(currentDf.first);
                    currentDf.second->appendCovisibility(newDf.first);
                }
            }
            // Erase duplicated word pointers
            newDf.second->eraseWord(_word);
        }
    }

    template <typename PointType_>
    inline bool Word<PointType_>::eraseProjection(int _dfId){
        if (projections.find(_dfId) != projections.end())
        {
            projections.erase(_dfId);
            idxInDf.erase(_dfId);
            dfIds.erase(std::remove(dfIds.begin(), dfIds.end(), _dfId), dfIds.end());
            return true;
        }
        else
        {
            return false;
        }
    }
    
    template <typename PointType_>
    inline void Word<PointType_>::updateNormal(){
        Eigen::Vector3f wordPos(point[0],point[1],point[2]);
            Eigen::Vector3f normal;
            int nClust=0;
            for(auto& df:dfMap){
                Eigen::Vector3f position = df->pose().block<3,1>(0,3);
                Eigen::Vector3f partialWordNormal = wordPos - position;
                normal = normal + partialWordNormal/partialWordNormal.norm();
                nClust++;
            }
            normalVector=normal/nClust;
    }

    template <typename PointType_>
    inline void Word<PointType_>::checkProjections(){
        updateNormal();
        Eigen::Vector3f wordPos(point[0],point[1],point[2]);
        for(auto& df:dfMap){
            Eigen::Vector3f position = df->pose().block<3,1>(0,3);
            Eigen::Vector3f partialWordNormal = wordPos - position;
            partialWordNormal = partialWordNormal/partialWordNormal.norm();
            Eigen::Vector3f wordNormalMean = normalVector/normalVector.norm();
            if(cos(45*M_PI/180)<abs(partialWordNormal.dot(wordNormalMean))){
                eraseProjection(df->id());
            }
        }
    }
} // namespace mico 