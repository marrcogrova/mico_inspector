////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_GUI_H_
#define RGBDSLAM_GUI_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

#include <thread>
#include <mutex>

namespace rgbd {
	/// Tool for visualization
	class Gui {
	public:		// Static interface
		static void init(unsigned _nViewports);
		static Gui *get();
		static void end();

	public:		// Public interface
		void showCloud(const pcl::PointCloud<pcl::PointXYZ> &_cloud, std::string _tag, unsigned _pointSize = 1, unsigned _viewportIndex = 0);
        void showCloud(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, std::string _tag, unsigned _pointSize = 1, unsigned _viewportIndex = 0);
        void showCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &_cloud, std::string _tag, unsigned _pointSize = 1, unsigned _viewportIndex = 0);

		void showCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud, std::string _tag, unsigned _pointSize = 1, int _nNormals = 100, unsigned _viewportIndex = 0);
		void showCloud(const pcl::PointCloud<pcl::PointNormal> &_cloud, std::string _tag, unsigned _pointSize = 1, int _nNormals = 100, unsigned _viewportIndex = 0);

        template<typename PointType_>
        void showCloud(const pcl::PointCloud<PointType_> &_cloud, std::string _tag, unsigned _pointSize = 1, unsigned _viewportIndex = 0);

		void showSurface(const pcl::PointCloud<pcl::PointXYZ> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha = 1, double _r = 1, double _g = 1, double _b = 1, unsigned _viewport = 0);
        void showSurface(const pcl::PointCloud<pcl::PointNormal>& _cloud, const std::vector<pcl::Vertices>& _faces, const std::string & _name, double _alpha = 1, double _r = 1, double _g = 1, double _b = 1, unsigned _viewport = 0);
        void showSurface(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha = 1, unsigned _viewport = 0);

        /// Change background
        void backgroundColor(double _r, double _g, double _b);

        /// Remove all the elements in the given viewport index.
		void clean(unsigned _viewportIndex);

        /// Remove from all the viewpoints of the 3d viewers the elements with given name.
        void clean(std::string _cloudName);

        /// Returns true if "esc" key is pressed in the 3d viewer and false if not.
        /// \return true if "esc" key is pressed in the 3d viewer and false if not.
		bool hasReceivedStopSignal();

        /// Register a custom callback with structure "void(const pcl::visualization::KeyboardEvent &, void*)"
        ///
        /// \code
        ///     rgbd::Gui::get()->registerCallback([](void(const pcl::visualization::KeyboardEvent & _event, void* _ptr){
        ///         if(event.keyDown() ....){
        ///             //DO STUFF
        ///         }
        ///     });
        /// \code
        ///
		void registerCallback(std::function<void(const pcl::visualization::KeyboardEvent &, void*)> _callback);

		void drawSphere(double _centroid[3], double _radius, std::string _name, unsigned _viewportIndex = 0);
        void drawSphere(double _centroid[3], double _radius, std::string _name, int r=255, int g=255, int b=255, int a=255, unsigned _viewportIndex=0);

        void drawCone(double _initPoint[3], double _endPoint[3], double _aperture, std::string _name, unsigned _viewportIndex = 0);

		void drawEllipse(const double _semiAxis[3], const Eigen::Affine3d &_transformation, std::string _tag, unsigned _n = 10, double r = 1, double g = 1, double b = 1, unsigned _pointSize = 1, unsigned _viewport = 0);

        void drawArrow(const pcl::PointNormal &_point, std::string _tag, double _r = 1, double _g = 1, double _b = 1, unsigned _lineWidth = 1, unsigned _viewport = 0);

        void drawCube(double _minX, double _minY, double _minZ, double _maxX, double _maxY, double _maxZ, std::string _tag, double _r = 1, double _g = 1, double _b = 1, unsigned _lineWidth = 1, unsigned _viewport = 0);

        void drawCoordinate(const Eigen::Matrix4f &_coordinate, double _size =0.25,  unsigned _viewportIndex = 0);

		template<typename FunctionType_, typename DataType_>
		void customPlot(FunctionType_ &_function, DataType_ &_data);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> & getViewer(){return mViewer;}

        /// \brief blocking call to pause app execution
        void pause();

	private:	// Private methods
		Gui(unsigned _nViewports);
		~Gui();

		void callbackKeyboard3dViewer(const pcl::visualization::KeyboardEvent &_event, void* _ptrViewer);
		void displayThreadBody();

		template<typename PointType_>
		void addCloudToViewer(const pcl::PointCloud<PointType_> &_cloud, std::string _tag, unsigned _pointSize = 1, unsigned _viewportIndex = 0);

		template<typename PointType_>
		void addCloudToViewerNormals(const pcl::PointCloud<PointType_> &_cloud, std::string _tag, unsigned _pointSize = 1, int _nNormals = 100, unsigned _viewportIndex = 0);

		template<typename PointType_>
		void addSurface(const pcl::PointCloud<PointType_> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha = 1, double _r = 1, double _g = 1, double _b = 1, unsigned _viewport = 0);

		template<typename PointType_>
		void addSurface(const pcl::PointCloud<PointType_> &_cloud, const std::vector<pcl::Vertices> &_faces, const std::string &_name, double _alpha = 1, unsigned _viewport = 0);


        void create1Viewports();
        void create2Viewports();
        void create3Viewports();
        void create4Viewports();
        void createNViewports();
	private:	// Members
		boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
		std::vector<int> mViewportIndexes;

		std::thread *mDisplayThread;
		std::mutex	mSecureMutex;
		bool receivedStopSignal = false;

		std::vector<std::string>	mCloudsToClean;
		std::vector<int>			mViewportsToClean;

		std::function<void(const pcl::visualization::KeyboardEvent &, void*)> mUserCallback = [](const pcl::visualization::KeyboardEvent &, void*) {};

		struct DrawData {
			std::string mName;
			unsigned mPointSize;
			unsigned mViewport;
			int mNormals;
			double r, g, b, alpha;

		};

		typedef std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, DrawData>			DrawDataXYZ;
		typedef std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, DrawData>			DrawDataXYZRGB;
        typedef std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, DrawData>			DrawDataXYZRGBA;

		typedef std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, DrawData>	DrawDataXYZRGBNormal;
		typedef std::pair<pcl::PointCloud<pcl::PointNormal>::Ptr, DrawData>			DrawDataNormal;
		typedef std::pair<pcl::PointNormal, DrawData>								DrawDataArrow;
		typedef std::pair<std::vector<pcl::Vertices>, DrawDataXYZ>					DrawDataSurface;
		typedef std::pair<std::vector<pcl::Vertices>, DrawDataXYZRGB>				DrawDataSurfaceRGB;
		typedef std::pair<pcl::ModelCoefficients, DrawData>							DrawDatashape;
        struct DrawDataCoordinate{
            Eigen::Matrix4f cs;
            double size;
            int viewport;
        };

        std::vector<DrawDataCoordinate, Eigen::aligned_allocator<DrawDataCoordinate>> mQueueCoordinates;

		std::vector<DrawDataXYZ>			mQueueXYZ;
		std::vector<DrawDataXYZRGB>			mQueueXYZRGB;
        std::vector<DrawDataXYZRGBA>		mQueueXYZRGBA;

		std::vector<DrawDataXYZRGBNormal>	mQueueXYZRGBNormal;
		std::vector<DrawDataNormal>			mQueueXYZNormal;
		std::vector<DrawDataArrow>			mQueueArrow;
		std::vector<DrawDataSurface>		mQueueSurfaces;
		std::vector<DrawDataSurfaceRGB>		mQueueSurfacesRGB;
		std::vector<DrawDatashape>			mQueueShapes;
		std::vector<std::function<void()>>	mQueueCustomDraw;

        bool mPause = false;

		static Gui *mInstance;
	};
}	//	namespace rgbd

#include "Gui.inl"

#endif	//	RGBDSLAM_GUI_H_
