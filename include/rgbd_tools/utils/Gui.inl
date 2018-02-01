//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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
    void Gui::showCloud(const pcl::PointCloud<PointType_> &_cloud, std::string _tag, unsigned _pointSize, unsigned _viewportIndex){
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.resize(_cloud.size());
        for(int i=0; i < _cloud.size(); i++){
            cloud[i].x = _cloud[i].x;
            cloud[i].y = _cloud[i].y;
            cloud[i].z = _cloud[i].z;
            cloud[i].rgb = ((int)_cloud[i].r) << 16 | ((int)_cloud[i].g) << 8 | ((int)_cloud[i].b);
            cloud[i].r = _cloud[i].r;
            cloud[i].g = _cloud[i].g;
            cloud[i].b = _cloud[i].b;
        }

        mSecureMutex.lock();
        mQueueXYZRGB.push_back(DrawDataXYZRGB(cloud.makeShared(), DrawData({ _tag, _pointSize, _viewportIndex, 0 ,0,0,0,0 })));
        mSecureMutex.unlock();
    }
}	//	namespace rgbd
