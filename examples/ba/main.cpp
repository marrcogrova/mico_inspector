#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>

using namespace g2o;
using namespace Eigen;
using namespace std;

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

int main()
{

    double OUTLIER_RATIO = 0.0;
    int PIXEL_NOISE = 0;

    //------------------
    // g2o::SparseOptimizer optimizer;
    // optimizer.setVerbose(false);
    // std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    // linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

    // g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    // );
    
    // optimizer.setAlgorithm(solver);

    //------
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();


    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );

    optimizer.setAlgorithm(solver);
    //----------------
    pcl::visualization::PCLVisualizer viewer("visualizer");

    vector<Vector3d> true_points;
    pcl::PointCloud<pcl::PointXYZRGB> oriCloud;
    pcl::PointCloud<pcl::PointXYZRGB> noiseCloud;
    pcl::PointCloud<pcl::PointXYZRGB> optimizedCloud;
    for (size_t i = 0; i < 500; ++i)
    {
        true_points.push_back(Vector3d((Sample::uniform() - 0.5) * 3,
                                       Sample::uniform() - 0.5,
                                       Sample::uniform() + 3));

        pcl::PointXYZRGB p(0, 0, 255);
        p.x = true_points.back()[0];
        p.y = true_points.back()[1];
        p.z = true_points.back()[2];
        oriCloud.push_back(p);
    }

    viewer.addPointCloud<pcl::PointXYZRGB>(oriCloud.makeShared(), "ori");

    double focal_length = 1000.;
    Vector2d principal_point(320., 240.);

    vector<g2o::SE3Quat,
           aligned_allocator<g2o::SE3Quat>>
        true_poses;
    g2o::CameraParameters *cam_params = new g2o::CameraParameters(focal_length, principal_point, 0.);
    cam_params->setId(0);

    // SET KEYFRAME VERTICES
    for (size_t i = 0; i < 15; i++)
    {
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        Vector3d trans(i*0.1-1.,0,0);

        Eigen::Quaterniond q;
        q.setIdentity();
        
        Matrix3d rot = Matrix3d::Identity(); 
        rot =   Eigen::AngleAxisd(  ((double)rand())/RAND_MAX*0.1*(i<2?0:1), Vector3d::UnitX())*
                Eigen::AngleAxisd(  ((double)rand())/RAND_MAX*0.1*(i<2?0:1), Vector3d::UnitY())*
                Eigen::AngleAxisd(  ((double)rand())/RAND_MAX*0.1*(i<2?0:1), Vector3d::UnitZ());

        Eigen::Quaterniond qnoise(rot);
        
        g2o::SE3Quat pose(  qnoise, 
                            trans + Vector3d(((double) rand())/RAND_MAX*0.1*(i<2?0:1), 0, 0)
                            );
        vSE3->setEstimate(pose);
        vSE3->setId(i);
        vSE3->setFixed(i < 2);
        optimizer.addVertex(vSE3);
        
        true_poses.push_back(g2o::SE3Quat(q, trans));

        viewer.addCoordinateSystem(0.1, Eigen::Affine3f(pose.to_homogeneous_matrix().cast<float>()), "cs" + std::to_string(i));
    }

    const float thHuber = sqrt(5.991);

    std::cout << true_points.size() << std::endl;
    // SET MAP POINT VERTICES
    for (size_t i = 0; i < true_points.size(); i++) {
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        Vector3d noisePoint = true_points[i] + Vector3d(Sample::gaussian(0.3),
                                                        Sample::gaussian(0.3),
                                                        Sample::gaussian(0.3));
        vPoint->setEstimate(noisePoint);
        int id = i + 15;
        vPoint->setId(id);
        vPoint->setMarginalized(true);

        pcl::PointXYZRGB np(255, 0, 0);
        np.x = noisePoint[0];
        np.y = noisePoint[1];
        np.z = noisePoint[2];

        int num_obs = 0;
        for (size_t j = 0; j < true_poses.size(); ++j)
        {
            auto tp = true_poses.at(j).map(true_points.at(i));
            Vector2d z = {
                tp[0]/tp[2]*1000 + 320,
                tp[1]/tp[2]*1000 + 240,
                };
            // std::cout << z.transpose() << std::endl;
            if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480)
            {
                ++num_obs;
            }
        }

        // std::cout << "--------------" <<std::endl;
        // getchar();
        //SET EDGES
        if (num_obs >= 2)
        {
            noiseCloud.push_back(np);
            optimizer.addVertex(vPoint);
            for (size_t j = 0; j < true_poses.size(); ++j)
            {
                auto tp = true_poses.at(j).map(true_points.at(i));
                Vector2d z = {
                    tp[0]/tp[2]*1000 + 320,
                    tp[1]/tp[2]*1000 + 240,
                    };
                if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480)
                {
                    double sam = Sample::uniform();
                    if (sam < OUTLIER_RATIO)
                    {
                        z = Vector2d(Sample::uniform(0, 640),
                                     Sample::uniform(0, 480));
                    }
                    z += Vector2d(Sample::gaussian(PIXEL_NOISE),
                                  Sample::gaussian(PIXEL_NOISE));

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(j)));
                    e->setMeasurement(z);
                    e->setInformation(Eigen::Matrix2d::Identity());

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);

                    e->fx = 1000;
                    e->fy = 1000;
                    e->cx = 320;
                    e->cy = 240;

                    optimizer.addEdge(e);
                }
            }
        }
    }

        viewer.addPointCloud<pcl::PointXYZRGB>(noiseCloud.makeShared(), "noise");

        // cv::namedWindow("peje");
        // while(cv::waitKey(3) != 's'){    
        //    viewer.spinOnce(30);
        // }

        // Optimize!
        optimizer.setVerbose(true);
        optimizer.save("g2o_map_1.g2o");
        optimizer.initializeOptimization();
        optimizer.optimize(50);
        optimizer.save("g2o_map_2.g2o");
        // Recover optimized data

        //Keyframes
        for (size_t i = 0; i < 15; i++)
        {
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(i));
            g2o::SE3Quat pose = vSE3->estimate();

            Matrix4f eigen_pose = pose.to_homogeneous_matrix().cast<float>();

            std::cout << eigen_pose.block<3,1>(0,3).transpose() << std::endl;

            viewer.addCoordinateSystem(0.15, Eigen::Affine3f(pose.to_homogeneous_matrix().cast<float>()), "cs_opt" + std::to_string(i));
        }

        for (int i = 15; i < optimizer.vertices().size(); i++)
        {
            g2o::VertexSBAPointXYZ *v_p = dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertices()[i]);

            if(v_p == nullptr)
                continue;

            pcl::PointXYZRGB p(0, 255, 0);

            p.x = v_p->estimate()[0];
            p.y = v_p->estimate()[1];
            p.z = v_p->estimate()[2];
            optimizedCloud.push_back(p);
        }
        viewer.addPointCloud<pcl::PointXYZRGB>(optimizedCloud.makeShared(), "optimizedCloud");
    
        cv::namedWindow("peje");
        while(cv::waitKey(3) != 's'){    
           viewer.spinOnce(30);
        }
}