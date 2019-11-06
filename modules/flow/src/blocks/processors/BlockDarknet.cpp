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

#include <mico/flow/blocks/processors/BlockDarknet.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

namespace mico{

    BlockDarknet::BlockDarknet(){
        
        iPolicy_ = new Policy({"color","dataframe"});

        opipes_["color"] = new OutPipe("color");
        opipes_["entities"] = new OutPipe("entities");

        iPolicy_->registerCallback({"color"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        #ifdef HAS_DARKNET
                                        if(hasParameters_){
                                            cv::Mat image;
                                            // check data received
                                            try{
                                                image = std::any_cast<cv::Mat>(_data["color"]).clone();
                                            }catch(std::exception& e){
                                                std::cout << "Failure Darknet. " <<  e.what() << std::endl;
                                                idle_ = true;
                                                return;
                                            }

                                            // get image detections
                                            auto detections = detector_.detect(image);
                                            for(auto &detection: detections){
                                                if(detection[1]>0.3){
                                                    cv::Rect rec(detection[2], detection[3], detection[4] -detection[2], detection[5]-detection[3]);
                                                    //cv::putText(image, "Confidence" + std::to_string(detection[1]), cv::Point2i(detection[2], detection[3]),1,2,cv::Scalar(0,255,0));
                                                    cv::putText(image, "ObjectId: " + std::to_string(detection[0]), cv::Point2i(detection[2], detection[3]),1,2,cv::Scalar(0,255,0));
                                                    cv::rectangle(image, rec, cv::Scalar(0,255,0));
                                                }
                                            }

                                            // send image with detections
                                            if(opipes_["color"]->registrations() !=0 )
                                                opipes_["color"]->flush(image);

                                        }else{
                                            std::cout << "No weights and cfg provided to Darknet\n";
                                        }
                                        #endif
                                        idle_ = true;
                                    }
                                });

        iPolicy_->registerCallback({"dataframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        #ifdef HAS_DARKNET
                                        if(hasParameters_){
                                            cv::Mat image;
                                            std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>> df(new mico::DataFrame<pcl::PointXYZRGBNormal>());

                                            // check data received
                                            try{
                                                df = std::any_cast<std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>>>(_data["dataframe"]);
                                                image = df->left.clone();
                                                
                                            }catch(std::exception& e){
                                                std::cout << "Failure Darknet dataframe registration. " <<  e.what() << std::endl;
                                                idle_ = true;
                                                return;
                                            }
                                            std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>> entities;
                                            // get image detections
                                            auto detections = detector_.detect(image);
                                            int i = 0;
                                            for(auto &detection: detections){
                                                
                                                numEntities++;
                                                
                                                
                                                // std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>> e(new mico::Entity<pcl::PointXYZRGBNormal>(
                                                //     numEntities, df->id, detection[0], detection[1], {detection[2],detection[3],detection[4],detection[5]},
                                                //     pose, cloud, projections, descriptors));
                                                                                                
                                                //df->detections[i] = {detection[1],detection[2],detection[3],detection[4],detection[5]};
                                                //i++;
                                                //entities.push_back(e);
                                                numEntities++;
                                            }
                                            std::cout << "Darknet block: " <<  df->id << " --> num of detections: " << i << std::endl;                                            
                                            // send dataframe with detections
                                            if(opipes_["entities"]->registrations() !=0 )
                                                opipes_["entities"]->flush(entities);

                                        }else{
                                            std::cout << "No weights and cfg provided to Darknet\n";
                                        }
                                        #endif
                                        idle_ = true;
                                    }
                                });
    }

    bool BlockDarknet::computePCA(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _cloud){        

        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid<pcl::PointXYZRGBNormal>(*_cloud, pcaCentroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized<pcl::PointXYZRGBNormal>(*_cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                       ///    the signs are different and the box doesn't get correctly oriented in some cases.
        // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PCA<pcl::PointXYZRGBNormal> pca;
        pca.setInputCloud(_cloud);
        //pca.project(*cloud, *cloudPCAprojection);
        std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
        std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
        // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
    

        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::transformPointCloud(*_cloud, *cloudPointsProjected, projectionTransform);
        // Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZRGBNormal minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
        // Final transform
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox1", mesh_vp_2);
    }

    bool BlockDarknet::configure(std::unordered_map<std::string, std::string> _params){        
        #ifdef HAS_DARKNET
        std::string cfgFile;
        std::string weightsFile;
        for(auto &p: _params){
            if(p.first == "cfg"){
                cfgFile = p.second;
            }else if(p.first == "weights"){
                weightsFile = p.second;
            }
        }

        hasParameters_ = true;  
        if(detector_.init(cfgFile,weightsFile)){
            return true;
        }
        else{
            std::cout << "Detector: Bad input arguments\n";
            return false;
        }
        #else
        return false;
        #endif
    }
    
    std::vector<std::string> BlockDarknet::parameters(){
        return {"cfg","weights"};
    }


}
