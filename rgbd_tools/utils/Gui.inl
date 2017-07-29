////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <vtkPolyDataNormals.h>
#include <vtkVersion.h>
#include "Gui.h"

namespace rgbd {
	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Gui::addCloudToViewer(const pcl::PointCloud<PointType_>& _cloud, std::string  _tag, unsigned _pointSize, unsigned _viewportIndex) {
		if (mViewer->contains(_tag)) {
			mViewer->updatePointCloud<PointType_>(_cloud.makeShared(), _tag);
			mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _pointSize, _tag);
		}
		else {
			mViewer->addPointCloud<PointType_>(_cloud.makeShared(), _tag, mViewportIndexes[_viewportIndex]);
			mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _pointSize, _tag);
		}
	}



	//---------------------------------------------------------------------------------------------------------------------
	// Template specializations
	template<>
	inline void Gui::addCloudToViewerNormals<pcl::PointNormal>(const pcl::PointCloud<pcl::PointNormal>& _cloud, std::string  _tag, unsigned _pointSize, int _nNormals, unsigned _viewportIndex) {
		if (mViewer->contains(_tag)) {
			mViewer->removePointCloud(_tag + "normals", mViewportIndexes[_viewportIndex]);
			mViewer->removePointCloud(_tag, mViewportIndexes[_viewportIndex]);
		}

		if (_nNormals == 0)
			mViewer->addPointCloud<pcl::PointNormal>(_cloud.makeShared(), _tag, mViewportIndexes[_viewportIndex]);
		else
			mViewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(_cloud.makeShared(), _cloud.makeShared(), _nNormals, 0.1, _tag, mViewportIndexes[_viewportIndex]);
		mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _pointSize, _tag);

	}

	//---------------------------------------------------------------------------------------------------------------------
	template<>
	inline void Gui::addCloudToViewerNormals<pcl::PointXYZRGBNormal>(const pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud, std::string  _tag, unsigned _pointSize, int _nNormals, unsigned _viewportIndex) {
		if (mViewer->contains(_tag)) {
			mViewer->removePointCloud(_tag + "normals", mViewportIndexes[_viewportIndex]);
			mViewer->removePointCloud(_tag, mViewportIndexes[_viewportIndex]);
		}

		if (_nNormals == 0)
			mViewer->addPointCloud<pcl::PointXYZRGBNormal>(_cloud.makeShared(), _tag, mViewportIndexes[_viewportIndex]);
		else
			mViewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(_cloud.makeShared(), _cloud.makeShared(), _nNormals, 0.1, _tag, mViewportIndexes[_viewportIndex]);
		mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _pointSize, _tag);
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<>
	inline void rgbd::Gui::addSurface(const pcl::PointCloud<pcl::PointXYZ>& _cloud, const std::vector<pcl::Vertices>& _faces, const std::string &_name, double _alpha, double _r, double _g, double _b, unsigned _viewport) {
		mViewer->addPolygonMesh<pcl::PointXYZ>(_cloud.makeShared(), _faces, _name, mViewportIndexes[_viewport]);
		mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, _r, _g, _b, _name);
		mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, _alpha, _name);
		// 666 TODO use normals from surface computation.
#if VTK_MAJOR_VERSION > 5
		auto actor = (*mViewer->getCloudActorMap())[_name].actor;
		vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
		normals->SetInputConnection(actor->GetMapper()->GetInputAlgorithm()->GetOutputPort());
		vtkDataSetMapper::SafeDownCast(actor->GetMapper())->SetInputConnection(normals->GetOutputPort());
		actor->GetProperty()->SetInterpolationToGouraud();
#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<>
	inline void rgbd::Gui::addSurface(const pcl::PointCloud<pcl::PointXYZRGB>& _cloud, const std::vector<pcl::Vertices>& _faces, const std::string &_name, double _alpha, unsigned _viewport) {
		mViewer->addPolygonMesh<pcl::PointXYZRGB>(_cloud.makeShared(), _faces, _name, mViewportIndexes[_viewport]);
		mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, _alpha, _name);
		// 666 TODO use normals from surface computation.
#if VTK_MAJOR_VERSION > 5
		auto actor = (*mViewer->getCloudActorMap())[_name].actor;
		vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
		normals->SetInputConnection(actor->GetMapper()->GetInputAlgorithm()->GetOutputPort());
		vtkDataSetMapper::SafeDownCast(actor->GetMapper())->SetInputConnection(normals->GetOutputPort());
		actor->GetProperty()->SetInterpolationToGouraud();
#endif
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename FunctionType_, typename DataType_>
	inline void Gui::customPlot(FunctionType_ & _function, DataType_ &_data) {
		auto lambda = [&]() {
			_function(_data, mViewer);
		};

		mQueueCustomDraw.push_back(lambda);
	}

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void Gui::showCloud(const pcl::PointCloud<PointType_> &_cloud, std::string _tag, bool hasColor, unsigned _pointSize, unsigned _viewportIndex){
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        for(auto &p:_cloud){
            pcl::PointXYZRGB p2;
            p2.x = p.x;
            p2.y = p.y;
            p2.z = p.z;
            if(hasColor){
                p2.r = p.r;
                p2.g = p.g;
                p2.b = p.b;
            }
            cloud.push_back(p2);
        }

        mSecureMutex.lock();
        mQueueXYZRGB.push_back(DrawDataXYZRGB(cloud.makeShared(), DrawData({ _tag, _pointSize, _viewportIndex, 0 ,0,0,0,0 })));
        mSecureMutex.unlock();
    }
}	//	namespace rgbd
