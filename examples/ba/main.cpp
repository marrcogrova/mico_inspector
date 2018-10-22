// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//         this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//         notice, this list of conditions and the following disclaimer in the
//         documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <Eigen/StdVector>
#include <iostream>
#include <stdint.h>

#include <unordered_set>
#include <pcl/visualization/pcl_visualizer.h>

#include <rgbd_tools/map3d/BundleAdjuster_g2o.h>
#include <rgbd_tools/map3d/BundleAdjusterCvsba.h>
#include <rgbd_tools/map3d/ClusterFrames.h>
#include <rgbd_tools/map3d/Word.h>

typedef pcl::PointXYZRGBNormal PointType;

using namespace Eigen;
using namespace std;
using namespace rgbd;
class Sample
{
public:
    static int uniform(int from, int to);
    static double uniform();
    static double gaussian(double sigma);
};

static double uniform_rand(double lowerBndr, double upperBndr)
{
    return lowerBndr + ((double)std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

static double gauss_rand(double mean, double sigma)
{
    double x, y, r2;
    do
    {
        x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
        y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
        r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

int Sample::uniform(int from, int to)
{
    return static_cast<int>(uniform_rand(from, to));
}

double Sample::uniform()
{
    return uniform_rand(0., 1.);
}

double Sample::gaussian(double sigma)
{
    return gauss_rand(0., sigma);
}

int main(int argc, const char *argv[])
{
    if (argc < 2)
    {
        cout << endl;
        cout << "Please type: " << endl;
        cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
        cout << endl;
        cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
        cout << "OUTLIER_RATIO: probability of spuroius observation        (default: 0.0)" << endl;
        cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
        cout << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)" << endl;
        cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
        cout << endl;
        cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
        cout << endl;
        exit(0);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    double PIXEL_NOISE = atof(argv[1]);
    double OUTLIER_RATIO = 0.0;

    if (argc > 2)
    {
        OUTLIER_RATIO = atof(argv[2]);
    }

    bool ROBUST_KERNEL = false;
    if (argc > 3)
    {
        ROBUST_KERNEL = atoi(argv[3]) != 0;
    }
    bool STRUCTURE_ONLY = false;
    if (argc > 4)
    {
        STRUCTURE_ONLY = atoi(argv[4]) != 0;
    }

    bool DENSE = false;
    if (argc > 5)
    {
        DENSE = atoi(argv[5]) != 0;
    }
    int optType = 0;
    if (argc > 6)
    {
        optType = atoi(argv[6]);
    }

    BundleAdjuster<PointType, rgbd::DebugLevels::Debug> *ba = nullptr;
    if (optType == 0)
    {
        ba = new rgbd::BundleAdjusterCvsba<PointType, rgbd::DebugLevels::Debug>;
        std::cout << "Optimization algorithm: cvsba" << std::endl;
    }
    else if (optType == 1)
    {
        ba = new rgbd::BundleAdjuster_g2o<PointType, rgbd::DebugLevels::Debug>;
        std::cout << "Optimization algorithm: g2o" << std::endl;
    }

    if (ba != nullptr)
    {
        // Initializing optimization module
        ba->minError(0.000000001);
        ba->iterations(1000);
        ba->minAparitions(1);
        ba->minWords(1);
    }
    else
    {
        return -1;
    }

    cout << "PIXEL_NOISE: " << PIXEL_NOISE << endl;
    cout << "OUTLIER_RATIO: " << OUTLIER_RATIO << endl;
    cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
    cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY << endl;
    cout << "DENSE: " << DENSE << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudGt(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNoise(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOptimized(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::map<int, ClusterFrames<PointType>::Ptr> subset;
    std::map<int, Word<PointType>::Ptr> words;


    vector<Vector3d> true_points;
    for (size_t i = 0; i < 500; ++i)
    {
        Word<PointType>::Ptr w = Word<PointType>::Ptr(new Word<PointType>);
        words[i] = w;
        w->id = i;

        w->point = {            (float) ((Sample::uniform() - 0.5) * 3),
                                (float) (Sample::uniform() - 0.5),
                                (float) (Sample::uniform() + 3)};

        true_points.push_back(Vector3d( (double) w->point[0],
                                        (double) w->point[1],
                                        (double) w->point[2]));


        pcl::PointXYZRGB p(0, 0, 255);

        p.x = w->point[0];
        p.y = w->point[1];
        p.z = w->point[2];

        cloudGt->push_back(p);
    }

    viewer->addPointCloud(cloudGt, "cloudGt");
    cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_64FC1);
    intrinsics.at<double>(0, 0) = 1000.0;
    intrinsics.at<double>(1, 1) = 1000.0;
    intrinsics.at<double>(0, 2) = 320.0;
    intrinsics.at<double>(1, 2) = 240.0;

    cv::Mat coeff = cv::Mat::zeros(5, 1, CV_64FC1);
    coeff.at<double>(0, 0) = 0;
    coeff.at<double>(1, 0) = 0;
    coeff.at<double>(2, 0) = 0;
    coeff.at<double>(3, 0) = 0;
    coeff.at<double>(4, 0) = 0;

    double focal_length = 1000.;
    Vector2d principal_point(320., 240.);

    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat>> true_poses;
    g2o::CameraParameters *cam_params = new g2o::CameraParameters(focal_length, principal_point, 0.);
    cam_params->setId(0);

    for (size_t i = 0; i < 15; ++i)
    {
        Vector3f trans(i * 0.1f - 1.0f, 0.0f, 0.0f);
        Eigen::Quaternionf q;
        q.setIdentity();
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 1>(0, 3) = trans;

        g2o::SE3Quat poseG2o(q.cast<double>(), trans.cast<double>());
        true_poses.push_back(poseG2o);


        // Noise to pose
        pose.block<3, 1>(0, 3) += Vector3f(
            ((double)rand())/RAND_MAX*0.1*(i<2?0:1),
            ((double)rand())/RAND_MAX*0.1*(i<2?0:1),
            ((double)rand())/RAND_MAX*0.1*(i<2?0:1)
        );

        Matrix3f rot = Matrix3f::Identity();
        rot =   AngleAxisf(((double)rand())/RAND_MAX*0.1*(i<2?0:1), Vector3f::UnitX())*
                AngleAxisf(((double)rand())/RAND_MAX*0.1*(i<2?0:1), Vector3f::UnitY())*
                AngleAxisf(((double)rand())/RAND_MAX*0.1*(i<2?0:1), Vector3f::UnitZ());

        pose.block<3,3>(0,0) = rot*pose.block<3,3>(0,0);

        viewer->addCoordinateSystem(0.1, Eigen::Affine3f(pose), "cs" + std::to_string(i));

        DataFrame<PointType>::Ptr df = DataFrame<PointType>::Ptr(new DataFrame<PointType>);
        df->pose = pose;
        df->position = trans;
        df->orientation = q;

        df->intrinsic = intrinsics;
        df->coefficients = coeff;


        df->id = i;
        subset[i] = ClusterFrames<PointType>::Ptr(new ClusterFrames<PointType>(df, i));


        subset[i]->intrinsic = intrinsics;
        subset[i]->distCoeff = coeff;
    }

    std::cout << "Prepared " << words.size() << " words "<<std::endl;
    std::cout << "Prepared " << subset.size() << " clusters "<<std::endl;
    
    for (auto w : words) {
        Vector3d pointNoise (   (double) w.second->point[0] + Sample::gaussian(1),
                                (double) w.second->point[1] + Sample::gaussian(1),
                                (double) w.second->point[2] + Sample::gaussian(1));

        w.second->point = {(float) pointNoise[0],(float)  pointNoise[1],(float)  pointNoise[2]};

        pcl::PointXYZRGB p(255, 0, 0);
        p.x = pointNoise[0];
        p.y = pointNoise[1];
        p.z = pointNoise[2];
        cloudNoise->push_back(p);

        int num_obs = 0;
        for (size_t j = 0; j < true_poses.size(); ++j) {
            // std::cout << true_poses[j]<< std::endl;
            // std::cout << true_points[w.first].transpose()<< std::endl;
            Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points[w.first]));
            // std::cout << z.transpose() << std::endl;
            // std::cout << "----------------" << std::endl;
            // cv::waitKey();
            if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
                ++num_obs;
            }
        }

        // std::cout << "Adding word " << w.first << ". Nobs: " << num_obs<< ". Proj: ";
        if (num_obs >= 2) {
            for (size_t j = 0; j < true_poses.size(); ++j) {
                Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points[w.first]));
                // std::cout << z.transpose() << std::endl;
                if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
                    // std::cout << j << ", ";
                    auto cf = subset[j];
                    if  (!w.second->isInFrame(cf->frames[0]) cf->wordsReference.push_back(w.second);
                    if (!w.second->isInCluster(j)) subset[j]->wordsReference[w.first] = w.second;
                    
                    subset[j]->frames.push_back(j);

                    if (!w.second->isInFrame(j))
                        w.second->frames.push_back(j);
                    if (!w.second->isInCluster(j))
                        w.second->clusters.push_back(j);

                    double sam = Sample::uniform();
                    if (sam < OUTLIER_RATIO) {
                        z = Vector2d(       (double) Sample::uniform(0, 640),
                                            (double) Sample::uniform(0, 480));
                    }
                    z += Vector2d(          (double) Sample::gaussian(PIXEL_NOISE),
                                            (double) Sample::gaussian(PIXEL_NOISE));

                    w.second->projections[j] = {(float)z[0], 
                                                (float)z[1]};

                                                
                }
            }
        }else{
            // std::cout << "--no projections--";
        }
        // std::cout << std::endl;
    }


    ba->clusterframes(subset);
    if(ba->optimizeClusterframes()){
        std::cout << "Failed optimization" << std::endl;
    }

    for (auto &w : words)
    {
        pcl::PointXYZRGB p(0, 255, 0);

        p.x = w.second->point[0];
        p.y = w.second->point[1];
        p.z = w.second->point[2];

        cloudOptimized->push_back(p);
    }

    for(auto &cluster: subset){

        viewer->addCoordinateSystem(0.2, Eigen::Affine3f(cluster.second->pose), "cs_opt" + std::to_string(cluster.first));
    }

    viewer->addPointCloud(cloudNoise, "cloudNoise");
    viewer->addPointCloud(cloudOptimized, "cloudOptimized");

    while (true)
    {
        viewer->spinOnce(30);
    }
}