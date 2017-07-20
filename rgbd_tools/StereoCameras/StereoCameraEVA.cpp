////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Authors: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include "StereoCameraEVA.h"

#include <pcl/features/normal_3d.h>

using namespace pcl;
using namespace std;

namespace rgbd{
	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraEva::init(const cjson::Json &_json) {
		#if HAS_ARTEC_SDK
			if (_json.isNull()) { // Defaul camera
				// 666 TODO
			}
			else { // Choose camera
				// 666 TODO
			}

			mLastErrorCode = asdk::ErrorCode_OK;
	
			asdk::TRef<asdk::IArrayScannerId> scannersList;

			mLastErrorCode = asdk::enumerateScanners( &scannersList );
			if( mLastErrorCode != asdk::ErrorCode_OK ){
				cout << "[STEREO CAMERA][ARTEC EVA] Couldn't load Artec's scanner list" << endl;
				return false;
			}
			int scannerCount = scannersList->getSize();
			if( scannerCount == 0 ) {
				cout << "[STEREO CAMERA][ARTEC EVA] No Artec's scanners found" << endl;
				return false;
			}

			const asdk::ScannerId* idArray = scannersList->getPointer();
			const asdk::ScannerId& defaultScanner = idArray[0]; // just take the first available scanner
			cout << "[STEREO CAMERA][ARTEC EVA] Connecting to " << asdk::getScannerTypeName( defaultScanner.type ) << endl << " scanner " << defaultScanner.serial << endl;

			mLastErrorCode = asdk::createScanner( &mScanner, &defaultScanner );
			if( mLastErrorCode != asdk::ErrorCode_OK ) {
				cout << "[STEREO CAMERA][ARTEC EVA] Failed to instantiate scanner" << endl;
				return false;
			}

			cout << "[STEREO CAMERA][ARTEC EVA] Scanner: " << asdk::getScannerTypeName( defaultScanner.type ) << " initialized. Ready to capture!" << endl;
			return true;
		#else
			return false;
		#endif
	}


	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraEva::rgb(cv::Mat &_left, cv::Mat &_right, bool _undistort) {
		#if HAS_ARTEC_SDK
			if (mHasRgb) {
				_left = mRgb;
				return true;
			}
			else {
				return false;
			}
		#else
			return false;
		#endif  //  HAS_ARTEC_SDK
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraEva::grab() {
		#if HAS_ARTEC_SDK
			// Init variables.
			asdk::ErrorCode mLastErrorCode = asdk::ErrorCode_OK;

			// Create a frame processor to generate the scan.
			mLastErrorCode = mScanner->createFrameProcessor(&mProcessor);
			
			return mLastErrorCode == asdk::ErrorCode_OK ? true : false;

		#else
			return false;
		#endif  //  HAS_ARTEC_SDK
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraEva::cloud(PointCloud<PointXYZ>& _cloud) {
		#if HAS_ARTEC_SDK
		// Try to scann from device.
		mFrame = NULL;
		mLastErrorCode = mScanner->capture( &mFrame, false ); // with texture
		if( mLastErrorCode == asdk::ErrorCode_OK ) {
			// Reconstruct mesh from frame.
			mMesh = NULL;			
			mProcessor->reconstructMesh( &mMesh, mFrame );
			if(!mMesh)
				return false;
			if( mLastErrorCode == asdk::ErrorCode_OK ) {
				// Get all data to transform mesh to pointcloud.
				auto points = mMesh->getPoints();
				auto pointsPointer = points->getPointer();

				// Parse mesh points into PCL point cloud.
				_cloud.resize(points->getSize());
				#pragma omp parallel for
					for (int i = 0; i < points->getSize(); i++) {
						auto p = pointsPointer[i];
						_cloud[i] = PointXYZ(p.x, p.y, p.z);
					}

				return true;
			}
		}
		#endif
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraEva::cloud(PointCloud<PointXYZRGB>& _cloud) {
		#if HAS_ARTEC_SDK
			// Try to scann from device.
			mFrame = NULL;
			mLastErrorCode = mScanner->capture( &mFrame, true ); // with texture
			if( mLastErrorCode == asdk::ErrorCode_OK ) {
				// Reconstruct mesh from frame.
				mMesh = NULL;			
				mProcessor->reconstructAndTexturizeMesh( &mMesh, mFrame );
				if(!mMesh)
					return false;

				if( mLastErrorCode == asdk::ErrorCode_OK ) {
					// Get all data to transform mesh to pointcloud.
					auto points = mMesh->getPoints();
					auto pointsPointer = points->getPointer();

					auto texture = mFrame->getTexture();
					auto pointCoordsUV = mMesh->getUVCoordinates()->getPointer();

					uchar * ptr = (uchar *)texture->getPointer();

					mRgb = cv::Mat(cv::Size(texture->getWidth(), texture->getHeight()), CV_8UC1, ptr);
					cv::cvtColor(mRgb, mRgb, CV_BayerGB2RGB);
					mHasRgb = true;

					// Parse mesh points into PCL point cloud.
					_cloud.resize(points->getSize());
					#pragma omp parallel for
						for (int i = 0; i < points->getSize(); i++) {
							auto texCords = pointCoordsUV[i];
							int scaledU = texture->getHeight() - (texCords.u)*texture->getHeight();
							int scaledV = (texCords.v)*texture->getWidth();
							if ( scaledV > texture->getWidth() || scaledU > texture->getHeight() || scaledV < 0 || scaledU < 0) {
								//cout << "Error maping point (u,v)=(" << texCords.u << ", " << texCords.v << ") -> (su,sv)=(" << scaledU << ", " << scaledV << ")" << endl;
								continue;
							}
							auto color = mRgb.at<cv::Vec3b>(scaledU, scaledV);
							auto artecPoint = pointsPointer[i];
							PointXYZRGB colorPoint(	color[2], color[1], color[0]);
							colorPoint.x = artecPoint.x/1000;
							colorPoint.y = artecPoint.y/1000;
							colorPoint.z = artecPoint.z/1000;

							_cloud[i] = (colorPoint);
						}
					return true;
				}
			}
		#endif
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraEva::cloud(PointCloud<PointNormal> &_cloud) {
		#if HAS_ARTEC_SDK
			// Init variables.
			asdk::ErrorCode mLastErrorCode = asdk::ErrorCode_OK;

			asdk::TRef<asdk::IFrame> frame;
			asdk::TRef<asdk::IFrameMesh> mesh;
			asdk::TRef<asdk::IFrameProcessor> processor;

			// Create a frame processor to generate the scan.
			mLastErrorCode = mScanner->createFrameProcessor( &processor );
			if( mLastErrorCode == asdk::ErrorCode_OK ) {
				// Try to scann from device.
				frame = NULL;
				mLastErrorCode = mScanner->capture( &frame, true ); // with texture.
				if( mLastErrorCode == asdk::ErrorCode_OK ) {
					// Reconstruct mesh from frame.
					mesh = NULL;			
					processor->reconstructMesh( &mesh, frame );
					if(!mesh)
						return false;

					if( mLastErrorCode == asdk::ErrorCode_OK ) {
						// Get all data to transform mesh to pointcloud.
						auto points = mesh->getPoints();
						auto pointsPointer = points->getPointer();

						artec::sdk::base::Point3F * pointNormals;

						if (!mesh->hasPointsNormals()) {
							mesh->calculate(artec::sdk::base::CalculateMode::CM_PointsNormals_Default);
						}

						pointNormals = mesh->getPointsNormals()->getPointer();

						// Parse mesh points into PCL point cloud.
						_cloud.resize(points->getSize());
						#pragma omp parallel for
							for (int i = 0; i < points->getSize(); i++) {
								auto artecPoint = pointsPointer[i];
								PointNormal point;

								point.x = artecPoint.x/1000;
								point.y = artecPoint.y/1000;
								point.z = artecPoint.z/1000;
								point.normal_x = pointNormals[i].x;
								point.normal_y = pointNormals[i].y;
								point.normal_z = pointNormals[i].z;

								_cloud[i] = point;
							}
						return true;
					}
				}
			}
		#endif
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraEva::cloud(PointCloud<PointXYZRGBNormal> &_cloud) {
	#if HAS_ARTEC_SDK
		// Init variables.
		asdk::ErrorCode mLastErrorCode = asdk::ErrorCode_OK;

		asdk::TRef<asdk::IFrame> frame;
		asdk::TRef<asdk::IFrameMesh> mesh;
		asdk::TRef<asdk::IFrameProcessor> processor;

		// Create a frame processor to generate the scan.
		mLastErrorCode = mScanner->createFrameProcessor( &processor );
		if( mLastErrorCode == asdk::ErrorCode_OK ) {
			// Try to scann from device.
			frame = NULL;
			mLastErrorCode = mScanner->capture( &frame, true ); // with texture.
			if( mLastErrorCode == asdk::ErrorCode_OK ) {
				// Reconstruct mesh from frame.
				mesh = NULL;			
				processor->reconstructAndTexturizeMesh( &mesh, frame );
				if(!mesh)
					return false;

				if( mLastErrorCode == asdk::ErrorCode_OK ) {
					// Get all data to transform mesh to pointcloud.
					auto points = mesh->getPoints();
					auto pointsPointer = points->getPointer();

					auto texture = frame->getTexture();
					auto pointCoordsUV = mesh->getUVCoordinates()->getPointer();
					artec::sdk::base::Point3F * pointNormals;

					if (!mesh->hasPointsNormals()) {
						mesh->calculate(artec::sdk::base::CalculateMode::CM_PointsNormals_Default);
					}

					pointNormals = mesh->getPointsNormals()->getPointer();


					uchar * ptr = (uchar *)texture->getPointer();
					cv::Mat cvTex(cv::Size(texture->getWidth(), texture->getHeight()), CV_8UC1, ptr);
					cv::cvtColor(cvTex, cvTex, CV_BayerGB2RGB);

					// Parse mesh points into PCL point cloud.
					_cloud.resize(points->getSize());
					#pragma omp parallel for
						for (int i = 0; i < points->getSize(); i++) {
							auto texCords = pointCoordsUV[i];
							int scaledU = texture->getHeight() - (texCords.u)*texture->getHeight();
							int scaledV = (texCords.v)*texture->getWidth();
							if ( scaledV >= texture->getWidth() || scaledU >= texture->getHeight() || scaledV < 0 || scaledU < 0) {
								//cout << "Error maping point (u,v)=(" << texCords.u << ", " << texCords.v << ") -> (su,sv)=(" << scaledU << ", " << scaledV << ")" << endl;
								continue;
							}
							auto color = cvTex.at<cv::Vec3b>(scaledU, scaledV);
							auto artecPoint = pointsPointer[i];
							PointXYZRGBNormal point;

							point.x = artecPoint.x/1000;
							point.y = artecPoint.y/1000;
							point.z = artecPoint.z/1000;
							point.r = color[2];
							point.g = color[1];
							point.b = color[0];
							point.normal_x = pointNormals[i].x;
							point.normal_y = pointNormals[i].y;
							point.normal_z = pointNormals[i].z;

							_cloud[i] = point;
						}
					return true;
				}
			}
		}
		#endif
		return false;
	}
}	//	namespace rgbd