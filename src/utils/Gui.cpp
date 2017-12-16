////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#include <rgbd_tools/utils/Gui.h>
#include <pcl/common/transforms.h>

namespace rgbd {
	//---------------------------------------------------------------------------------------------------------------------
	// Static data initialization
	Gui* Gui::mInstance = nullptr;

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::init(unsigned _nViewports) {
		mInstance = new Gui(_nViewports);
	}

	//---------------------------------------------------------------------------------------------------------------------
	Gui * Gui::get() {
		return mInstance;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::end() {
		delete mInstance;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::showCloud(const pcl::PointCloud<pcl::PointXYZ>& _cloud, std::string _tag, unsigned _pointSize, unsigned _viewportIndex) {
		mSecureMutex.lock();
		mQueueXYZ.push_back(DrawDataXYZ(_cloud.makeShared(), DrawData({ _tag, _pointSize, _viewportIndex, 0 ,0,0,0 })));
		mSecureMutex.unlock();
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::showCloud(const pcl::PointCloud<pcl::PointXYZRGB>& _cloud, std::string _tag, unsigned _pointSize, unsigned _viewportIndex) {
		mSecureMutex.lock();
		mQueueXYZRGB.push_back(DrawDataXYZRGB(_cloud.makeShared(), DrawData({ _tag, _pointSize, _viewportIndex, 0 ,0,0,0,0 })));
		mSecureMutex.unlock();
	}

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::showCloud(const pcl::PointCloud<pcl::PointXYZRGBA>& _cloud, std::string _tag, unsigned _pointSize, unsigned _viewportIndex) {
        mSecureMutex.lock();
        mQueueXYZRGBA.push_back(DrawDataXYZRGBA(_cloud.makeShared(), DrawData({ _tag, _pointSize, _viewportIndex, 0 ,0,0,0,0 })));
        mSecureMutex.unlock();
    }


	//---------------------------------------------------------------------------------------------------------------------
	void Gui::showCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud, std::string _tag, unsigned _pointSize, int _nNormals, unsigned _viewportIndex) {
		mSecureMutex.lock();
		mQueueXYZRGBNormal.push_back(DrawDataXYZRGBNormal(_cloud.makeShared(), DrawData({ _tag, _pointSize, _viewportIndex, _nNormals ,0,0,0,0 })));
		mSecureMutex.unlock();
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::showCloud(const pcl::PointCloud<pcl::PointNormal>& _cloud, std::string _tag, unsigned _pointSize, int _nNormals, unsigned _viewportIndex) {
		mSecureMutex.lock();
		mQueueXYZNormal.push_back(DrawDataNormal(_cloud.makeShared(), DrawData({ _tag, _pointSize, _viewportIndex, _nNormals ,0,0,0,0 })));
		mSecureMutex.unlock();
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::showSurface(const pcl::PointCloud<pcl::PointXYZ>& _cloud, const std::vector<pcl::Vertices>& _faces, const std::string & _name, double _alpha, double _r, double _g, double _b, unsigned _viewport) {
		mSecureMutex.lock();
		DrawDataXYZ drawData = DrawDataXYZ(_cloud.makeShared(), DrawData({ _name, 0, _viewport, 0 ,_r,_g,_b,_alpha }));
		mQueueSurfaces.push_back(DrawDataSurface(_faces, drawData));
		mSecureMutex.unlock();
	}

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::showSurface(const pcl::PointCloud<pcl::PointNormal>& _cloud, const std::vector<pcl::Vertices>& _faces, const std::string & _name, double _alpha, double _r, double _g, double _b, unsigned _viewport) {
        pcl::PointCloud<pcl::PointXYZ> justPoints;
        for(auto &p:_cloud){
            justPoints.push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
        mSecureMutex.lock();
        DrawDataXYZ drawData = DrawDataXYZ(justPoints.makeShared(), DrawData({ _name, 0, _viewport, 0 ,_r,_g,_b,_alpha }));
        mQueueSurfaces.push_back(DrawDataSurface(_faces, drawData));
        mSecureMutex.unlock();
    }

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::showSurface(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha, unsigned _viewport) {
		mSecureMutex.lock();
		DrawDataXYZRGB drawData = DrawDataXYZRGB(_cloud.makeShared(), DrawData({ _name, 0, _viewport, 0 ,1.0,1.0,1.0,_alpha }));
		mQueueSurfacesRGB.push_back(DrawDataSurfaceRGB(_faces, drawData));
        mSecureMutex.unlock();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::backgroundColor(double _r, double _g, double _b) {
        for (unsigned viewport = 0; viewport < mViewportIndexes.size(); viewport++) {
            mSecureMutex.lock();
            mViewer->setBackgroundColor(_r, _g, _b, mViewportIndexes[viewport]);
            mSecureMutex.unlock();
        }
    }

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::clean(unsigned _viewportIndex) {
		mSecureMutex.lock();
		// Check if viewport index is valid
		if (_viewportIndex < mViewportIndexes.size()) {
			// Mark viewport to be 
			bool pendingToRemove = false;
			for (auto viewport : mViewportsToClean) {
				if (viewport == _viewportIndex) {
					pendingToRemove = true;
					break;
				}
			}

			if (!pendingToRemove)
				mViewportsToClean.push_back(_viewportIndex);
		}
		mSecureMutex.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));	// 666 TODO non efficient!
	}

	//---------------------------------------------------------------------------------------------------------------------
    void Gui::clean(std::string _cloudName) {
		mSecureMutex.lock();
		bool pendingToRemove = false;
		for (auto viewport : mCloudsToClean) {
			if (viewport == _cloudName) {
				pendingToRemove = true;
				break;
			}
		}

		if (!pendingToRemove)
			mCloudsToClean.push_back(_cloudName);

		mSecureMutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));	// 666 TODO non efficient!
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool Gui::hasReceivedStopSignal() {
		return receivedStopSignal;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::registerCallback(std::function<void(const pcl::visualization::KeyboardEvent &, void*)> _callback) {
		mUserCallback = _callback;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::drawSphere(double _centroid[3], double _radius, std::string _name, unsigned _viewportIndex) {
		pcl::ModelCoefficients coeff;
		coeff.values.push_back(_centroid[0]);
		coeff.values.push_back(_centroid[1]);
		coeff.values.push_back(_centroid[2]);
		coeff.values.push_back(_radius);

		mSecureMutex.lock();
		mQueueShapes.push_back(DrawDatashape(coeff, DrawData({ _name, 0, _viewportIndex, 0, 0, 0, 0, 0 })));
		mSecureMutex.unlock();
	}

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::drawCone(double _initPoint[3], double _endPoint[3], double _aperture, std::string _name, unsigned _viewportIndex) {
        pcl::ModelCoefficients coeff;
        coeff.values.push_back(_initPoint[0]);
        coeff.values.push_back(_initPoint[1]);
        coeff.values.push_back(_initPoint[2]);
        coeff.values.push_back(_endPoint[0]);
        coeff.values.push_back(_endPoint[1]);
        coeff.values.push_back(_endPoint[2]);
        coeff.values.push_back(_aperture);

        mSecureMutex.lock();
        mQueueShapes.push_back(DrawDatashape(coeff, DrawData({ _name, 0, _viewportIndex, 0, 0, 0, 0, 0 })));
        mSecureMutex.unlock();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::drawSphere(double _centroid[3], double _radius, std::string _name, int r, int g, int b, int a, unsigned _viewportIndex) {
        pcl::ModelCoefficients coeff;
        coeff.values.push_back(_centroid[0]);
        coeff.values.push_back(_centroid[1]);
        coeff.values.push_back(_centroid[2]);
        coeff.values.push_back(_radius);

        mSecureMutex.lock();
        mQueueShapes.push_back(DrawDatashape(coeff, DrawData({ _name, 0, _viewportIndex, 0, r/255.0, g/255.0, b/255.0, a/255.0 })));
        mSecureMutex.unlock();
    }

	//---------------------------------------------------------------------------------------------------------------------
    void Gui::drawEllipse(const float _semiAxis[3], const Eigen::Affine3f &_transformation, std::string _tag, unsigned _n, double _r, double _g, double _b, unsigned _pointSize, unsigned _viewport) {
		pcl::PointCloud<pcl::PointXYZRGB> ellipseCloud;

		// Compute points on ellipsion centered on origin.
		double incZ = _semiAxis[2] / _n;

		double z = -_semiAxis[2];
		while (z < _semiAxis[2]) {
			for (unsigned i = 0; i < _n; i++) {
				double r = 1 - z*z / _semiAxis[2];
				double x = r*cos(2 * M_PI / _n*i) * _semiAxis[0];
				double y = r*sin(2 * M_PI / _n*i) * _semiAxis[1];

				pcl::PointXYZRGB p(_r * 255, _g * 255, _b * 255);
				p.x = x;
				p.y = y;
				p.z = z;

				ellipseCloud.push_back(p);
			}
			z += incZ;
		}

		pcl::transformPointCloud(ellipseCloud, ellipseCloud, _transformation.matrix());

		showCloud(ellipseCloud, _tag, _pointSize, _viewport);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::drawArrow(const pcl::PointNormal &_point, std::string _tag, double _r, double _g, double _b, unsigned _lineWidth, unsigned _viewport) {
		DrawData data;
		data.r = _r;
		data.g = _g;
		data.b = _b;
		data.mName = _tag;
		data.mPointSize = _lineWidth;
		data.mViewport = _viewport;
		mSecureMutex.lock();
		mQueueArrow.push_back(DrawDataArrow(_point, data));
        mSecureMutex.unlock();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::drawCube(double _minX, double _minY, double _minZ, double _maxX, double _maxY, double _maxZ, std::string _tag, double _r , double _g , double _b , unsigned _lineWidth , unsigned _viewport){
        pcl::ModelCoefficients coeff;
        coeff.values.push_back(_minX);
        coeff.values.push_back(_minY);
        coeff.values.push_back(_minZ);
        coeff.values.push_back(_maxX);
        coeff.values.push_back(_maxY);
        coeff.values.push_back(_maxZ);

        mSecureMutex.lock();
        mQueueShapes.push_back(DrawDatashape(coeff, DrawData({ _tag, _lineWidth, _viewport, 0, _r, _g, _b, 255 })));
        mSecureMutex.unlock();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::drawCoordinate(const Eigen::Matrix4f &_coordinate, double _size ,  unsigned _viewportIndex ) {
        mQueueCoordinates.push_back({_coordinate, _size, (int) _viewportIndex});
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::pause() {
        mPause = true;
        while(mPause){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

	//---------------------------------------------------------------------------------------------------------------------
	// Private interface
	//---------------------------------------------------------------------------------------------------------------------
	Gui::~Gui() {
		receivedStopSignal = true;
		mDisplayThread->join();
	}

	//---------------------------------------------------------------------------------------------------------------------
	Gui::Gui(unsigned _nViewports) {
		std::cout << "[GUI] Initilizing Graphic interface" << std::endl;
		vtkObject::GlobalWarningDisplayOff();	// 666 TODO: could be dangerous

		for (unsigned i = 0; i < _nViewports; i++) {
			mViewportIndexes.push_back(i);
		}

		mDisplayThread = new std::thread(&Gui::displayThreadBody, this);
        while(mViewer == nullptr){
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
		std::cout << "[GUI] Graphic interface initialized" << std::endl;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void Gui::callbackKeyboard3dViewer(const pcl::visualization::KeyboardEvent & _event, void * _ptrViewer) {
		std::cout << "Catched key event: " << _event.getKeySym() << std::endl;
		if (_event.getKeySym() == "Escape" && _event.keyDown()) {
			// Stop the system.
			std::cout << "Received stop signal" << std::endl;
			receivedStopSignal = true;
        }else if(_event.getKeySym() == "space" && _event.keyDown()){
            mPause = false;
        }

		mUserCallback(_event, _ptrViewer);

	}
	//---------------------------------------------------------------------------------------------------------------------
	void Gui::displayThreadBody() {
		mViewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));


        if(mViewportIndexes.size() == 1){
            create1Viewports();
        }else if(mViewportIndexes.size() == 2){
            create2Viewports();
        }else if(mViewportIndexes.size() == 3){
            create3Viewports();
        }else if(mViewportIndexes.size() == 4){
            create4Viewports();
        }else if(mViewportIndexes.size() > 4){
            createNViewports();
        }

		mViewer->registerKeyboardCallback(&Gui::callbackKeyboard3dViewer, *this, (void*)&mViewer);
		while (!receivedStopSignal) {
			mSecureMutex.lock();
			// Clean pending viewports.
			if (mViewportsToClean.size() != 0) {
				for (auto viewport : mViewportsToClean) {
					mViewer->removeAllPointClouds(mViewportIndexes[viewport]);
					mViewer->removeAllShapes(mViewportIndexes[viewport]);
				}
				mViewportsToClean.clear();
			}

			// Clean pending clouds.
			if (mCloudsToClean.size() != 0) {
				for (auto cloudName : mCloudsToClean) {
					for (unsigned viewport = 0; viewport < mViewportIndexes.size(); viewport++) {
						mViewer->removePointCloud(cloudName, mViewportIndexes[viewport]);
						mViewer->removePointCloud(cloudName + "normals", mViewportIndexes[viewport]);
						mViewer->removeShape(cloudName, mViewportIndexes[viewport]);
						mViewer->removePolygonMesh(cloudName, mViewportIndexes[viewport]);
					}
				}
				mCloudsToClean.clear();
			}

			// Draw
            for (auto &toDraw : mQueueXYZ) {
				addCloudToViewer(*toDraw.first, toDraw.second.mName, toDraw.second.mPointSize, toDraw.second.mViewport);
			}
			mQueueXYZ.clear();

            for (auto &toDraw : mQueueXYZRGB) {
				addCloudToViewer(*toDraw.first, toDraw.second.mName, toDraw.second.mPointSize, toDraw.second.mViewport);
			}
			mQueueXYZRGB.clear();

            for (auto &toDraw : mQueueXYZRGBA) {
                addCloudToViewer(*toDraw.first, toDraw.second.mName, toDraw.second.mPointSize, toDraw.second.mViewport);
            }
            mQueueXYZRGBA.clear();

            for (auto &toDraw : mQueueXYZRGBNormal) {
				addCloudToViewer(*toDraw.first, toDraw.second.mName, toDraw.second.mPointSize, toDraw.second.mViewport);
				addCloudToViewerNormals(*toDraw.first, toDraw.second.mName + "normals", toDraw.second.mPointSize, toDraw.second.mNormals, toDraw.second.mViewport);
			}
			mQueueXYZRGBNormal.clear();

			for (auto toDraw : mQueueXYZNormal) {
				addCloudToViewer(*toDraw.first, toDraw.second.mName, toDraw.second.mPointSize, toDraw.second.mViewport);
				addCloudToViewerNormals(*toDraw.first, toDraw.second.mName + "normals", toDraw.second.mPointSize, toDraw.second.mNormals, toDraw.second.mViewport);
			}
			mQueueXYZNormal.clear();

            for (auto &toDraw : mQueueSurfaces) {
				auto data = toDraw.second.second;
				addSurface(*toDraw.second.first, toDraw.first, data.mName, data.alpha, data.r, data.g, data.b, data.mViewport);
			}
			mQueueSurfaces.clear();

            for (auto &toDraw : mQueueSurfacesRGB) {
				auto data = toDraw.second.second;
				addSurface(*toDraw.second.first, toDraw.first, data.mName, data.alpha, data.mViewport);
			}
			mQueueSurfacesRGB.clear();

            for (auto &toDraw : mQueueShapes) {
                if(toDraw.first.values.size() == 4){ // Sphere
                    pcl::PointXYZ p(toDraw.first.values[0], toDraw.first.values[1], toDraw.first.values[2]);
                    mViewer->addSphere( p,
                                        toDraw.first.values[3],
                                        toDraw.second.r, toDraw.second.g, toDraw.second.b,
                                        toDraw.second.mName,
                                        mViewportIndexes[toDraw.second.mViewport]);

                    mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, toDraw.second.alpha, toDraw.second.mName);
                }else if(toDraw.first.values.size() == 7){
                    mViewer->addCone( toDraw.first,
                                      toDraw.second.mName,
                                      mViewportIndexes[toDraw.second.mViewport]);

                    mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, toDraw.second.r, toDraw.second.g, toDraw.second.b,toDraw.second.mName);
                    mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, toDraw.second.alpha, toDraw.second.mName);
                }else if(toDraw.first.values.size() == 6){
                    mViewer->addCube(   toDraw.first.values[0],
                                        toDraw.first.values[3],
                                        toDraw.first.values[1],
                                        toDraw.first.values[4],
                                        toDraw.first.values[2],
                                        toDraw.first.values[5],
                                        toDraw.second.r, toDraw.second.g, toDraw.second.b,
                                      toDraw.second.mName,
                                      mViewportIndexes[toDraw.second.mViewport]);

                    mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, toDraw.second.mPointSize,toDraw.second.mName);
                    mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, toDraw.second.mPointSize,toDraw.second.mName);
                }
			}
			mQueueShapes.clear();

            for (auto &toDraw : mQueueCustomDraw) {
				toDraw();
			}
			mQueueCustomDraw.clear();

            for (auto &toDraw : mQueueArrow) {
				auto p1 = toDraw.first;
				auto p2 = toDraw.first;
				p2.x += p2.normal_x;
				p2.y += p2.normal_y;
				p2.z += p2.normal_z;
				mViewer->addLine(p1, p2, toDraw.second.r, toDraw.second.g, toDraw.second.b, toDraw.second.mName, toDraw.second.mViewport);
			}
			mQueueArrow.clear();

            int coordinateCounter = 0;
            for (auto &coordinate: mQueueCoordinates) {
                Eigen::Affine3f pose;
                pose.matrix() = coordinate.cs;
                mViewer->addCoordinateSystem(coordinate.size, pose, "coordinate_"+std::to_string(coordinateCounter++), coordinate.viewport);
            }
            mQueueCoordinates.clear();


			mViewer->spinOnce();
			mSecureMutex.unlock();

			std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::create1Viewports(){
        mViewer->createViewPort(0.0, 0, 1.0, 1.0, mViewportIndexes[0]);
        mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[0]);
        mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(0), mViewportIndexes[0]);

    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::create2Viewports(){
        for (unsigned i = 0; i < mViewportIndexes.size(); i++) {
            mViewer->createViewPort(1.0 / mViewportIndexes.size() * i, 0, 1.0 / mViewportIndexes.size() * (i + 1), 1.0, mViewportIndexes[i]);
            mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[i]);
            mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(i), mViewportIndexes[i]);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::create3Viewports(){
        mViewer->createViewPort(0.0, 0.5, 0.5, 1.0, mViewportIndexes[0]);
        mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[0]);
        mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(0), mViewportIndexes[0]);

        mViewer->createViewPort(0.5, 0.5, 1.0, 1.0, mViewportIndexes[1]);
        mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[1]);
        mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(1), mViewportIndexes[1]);

        mViewer->createViewPort(0, 0.0, 1.0, 0.5, mViewportIndexes[2]);
        mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[2]);
        mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(2), mViewportIndexes[2]);
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::create4Viewports(){
        mViewer->createViewPort(0, 0.5, 0.5, 1.0, mViewportIndexes[0]);
        mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[0]);
        mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(0), mViewportIndexes[0]);

        mViewer->createViewPort(0.5, 0.5, 1.0, 1.0, mViewportIndexes[1]);
        mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[1]);
        mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(1), mViewportIndexes[1]);

        mViewer->createViewPort(0.0, 0.0, 0.5, 0.5, mViewportIndexes[2]);
        mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[2]);
        mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(2), mViewportIndexes[2]);

        mViewer->createViewPort(0.5, 0.0, 1.0, 0.5, mViewportIndexes[3]);
        mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[3]);
        mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(3), mViewportIndexes[3]);
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Gui::createNViewports(){
        for (unsigned i = 0; i < mViewportIndexes.size(); i++) {
            mViewer->createViewPort(1.0 / mViewportIndexes.size() * i, 0, 1.0 / mViewportIndexes.size() * (i + 1), 1.0, mViewportIndexes[i]);
            mViewer->setBackgroundColor(1.0,1.0,1.0, mViewportIndexes[i]);
            mViewer->addCoordinateSystem(0.1, "XYZ_" + std::to_string(i), mViewportIndexes[i]);
            //mViewer->setCameraPosition(0.0, 0.2, 0.75, 0.0, 0.0, -1.0, 0.0, 1.0, 0.1, mViewportIndexes[i]);
        }
    }

	//---------------------------------------------------------------------------------------------------------------------
}	//	namespace rgbd
