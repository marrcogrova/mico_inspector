///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POINTPROB_H_
#define POINTPROB_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/impl/icp_nl.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/impl/integral_image_normal.hpp>

template<int ProbTypes_>
struct EIGEN_ALIGN16 PointProb{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_NORMAL4D;
    float curvature;
    int nProbs = ProbTypes_;
    float probs[ProbTypes_];
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
};                    // enforce SSE padding for correct memory alignment


POINT_CLOUD_REGISTER_POINT_STRUCT (PointProb<3>,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (float[3], probs, probs)
                                   )

#endif // POINTPROB_H_