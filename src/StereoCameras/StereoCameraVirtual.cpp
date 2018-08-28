////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#include <rgbd_tools/StereoCameras/StereoCameraVirtual.h>

#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

using namespace cv;
using namespace pcl;
using namespace std;

namespace rgbd {
	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::init(const cjson::Json &_json) {
		if (_json.isObject()) {
            mLeftImageFilePathTemplate = std::string(_json ["input"]["left"]);
            mRightImageFilePathTemplate = std::string(_json["input"]["right"]);
            mDepthImageFilePathTemplate = std::string(_json["input"]["depth"]);
            mPointCloudFilePathTemplate = std::string(_json["input"]["pointCloud"]);

            // Load Calibration files if path exist
            if(_json.contains("calibFile") && std::string(_json["calibFile"]) != ""){
                mHasCalibration = true;

                FileStorage fs((std::string)_json["calibFile"], FileStorage::READ);

                fs["MatrixLeft"]            >> mMatrixLeft;
                fs["DistCoeffsLeft"]        >> mDistCoefLeft;
                fs["MatrixRight"]           >> mMatrixRight;
                fs["DistCoeffsRight"]       >> mDistCoefRight;
                fs["Rotation"]              >> mRot;
                fs["Translation"]           >> mTrans;
                fs["DisparityToDepthScale"] >> mDispToDepth;

            }else{
                mHasCalibration = false;
            }

			return true;
		}
		else {
			std::cout << "[STEREO CAMERA][VIRTUAL] Virtual stereo camera couldn't be initialized" << std::endl;
			return false;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::rgb(Mat & _left, Mat & _right) {
		if (mLeftImageFilePathTemplate == "" && mRightImageFilePathTemplate == "") {
			return false;
		}

        _left = mLeft;
        _right = mRight;

        if (_right.rows == 0 && _left.rows == 0) {
			return false;
		}
		else {
            if (_right.rows == 0 || _left.rows == 0) {
				std::cout << "[STEREO CAMERA][VIRTUAL] Warning, this camera only provide one color image\n";
			}

			return true;
		}

	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::depth(Mat & _depth) {
		if (mDepthImageFilePathTemplate == "") {
			return false;
		}

        _depth = mDepth;

		if (_depth.rows == 0)
			return false;
		else
			return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(PointCloud<PointXYZ>& _cloud) {

		if (mPointCloudFilePathTemplate == "") {
			if (mDepthImageFilePathTemplate == "") {
				return false;
			}
            else {
                depthToPointcloud(mDepth, _cloud);
				return true;
			}
		}
		else {
			int indexEntryPoint = mPointCloudFilePathTemplate.find("%d");
			string imagePath = mPointCloudFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mPointCloudFilePathTemplate.substr(indexEntryPoint + 2);

			pcl::io::loadPCDFile(imagePath, _cloud);
			if (_cloud.size() != 0) {
				return true;
			}
		}
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			int indexEntryPoint = mPointCloudFilePathTemplate.find("%d");
			string imagePath = mPointCloudFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mPointCloudFilePathTemplate.substr(indexEntryPoint + 2);

			pcl::io::loadPCDFile(imagePath, _cloud);
			if (_cloud.size() != 0) {
				return true;
			}
        }else if(mDepthImageFilePathTemplate != ""){
            depthToPointcloud(mDepth, _cloud);
            return true;
        }
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointNormal> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			int indexEntryPoint = mPointCloudFilePathTemplate.find("%d");
			string imagePath = mPointCloudFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mPointCloudFilePathTemplate.substr(indexEntryPoint + 2);

			pcl::io::loadPCDFile(imagePath, _cloud);
			if (_cloud.size() != 0) {
				return true;
			}
		}
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud) {
		if (mPointCloudFilePathTemplate != "") {
			int indexEntryPoint = mPointCloudFilePathTemplate.find("%d");
			string imagePath = mPointCloudFilePathTemplate.substr(0, indexEntryPoint) + to_string(mFrameCounter) + mPointCloudFilePathTemplate.substr(indexEntryPoint + 2);

            pcl::io::loadPCDFile(imagePath, _cloud);
            if (_cloud.size() == 0) {
                return false;
            }else{
                pcl::PCLPointCloud2* header = new pcl::PCLPointCloud2;
                pcl::PCDReader reader;
                reader.readHeader(imagePath, *header);

                bool hasNormals = false;
                for(unsigned i = 0; i < header->fields.size(); i++){
                    if(header->fields[i].name.find("normal")!= std::string::npos){
                        hasNormals = true;
                    }
                }

                if(hasNormals){
                    return true;
                }else{
                    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
                    ne.setInputCloud(_cloud.makeShared());
                    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
                    ne.setMaxDepthChangeFactor(0.02f);
                    ne.setNormalSmoothingSize(10.0f);
                    ne.compute(_cloud);
                    return true;
                }
            }
        }else if (mDepthImageFilePathTemplate != "") {
            return false;   // 666 not implemented but should be possible
        }
        else {

            return false;
        }
    }
    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::leftCalibration(Mat &_intrinsic, Mat &_coefficients){
        _intrinsic = mMatrixLeft;
        _coefficients = mDistCoefLeft;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::rightCalibration(Mat &_intrinsic, Mat &_coefficients){
        _intrinsic = mMatrixRight;
        _coefficients = mDistCoefRight;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::extrinsic(Mat &_rotation, Mat &_translation){
        _rotation = mRot;
        _translation = mTrans;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation){
        _rotation = Eigen::Matrix3f((float*)mRot.data);
        _translation = Eigen::Vector3f((float*)mTrans.data);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mDispToDepth;
        return true;
    }

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::grab() {
		mFrameCounter++;

        if (mLeftImageFilePathTemplate != "") {
            int indexEntryPoint = mLeftImageFilePathTemplate.find("%d");
            auto imagePath = mLeftImageFilePathTemplate;
            imagePath.replace(indexEntryPoint, 2, std::to_string(mFrameCounter));
            mLeft = imread(imagePath);
        }

        if (mRightImageFilePathTemplate != "") {
            int indexEntryPoint = mRightImageFilePathTemplate.find("%d");
            auto imagePath = mRightImageFilePathTemplate;
            imagePath.replace(indexEntryPoint, 2, std::to_string(mFrameCounter));
            mRight = imread(imagePath.substr(0, imagePath.size() - 1));
        }

        if(mDepthImageFilePathTemplate != ""){
            int indexEntryPoint = mDepthImageFilePathTemplate.find("%d");
            auto imagePath = mDepthImageFilePathTemplate;
            imagePath.replace(indexEntryPoint, 2, std::to_string(mFrameCounter));
            mDepth = imread(imagePath, CV_LOAD_IMAGE_UNCHANGED);
        }


		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool StereoCameraVirtual::grab(uint index) {
		// mFrameCounter++;
		mFrameCounter = index;
		std::cout << "HERE" << std::endl;
        if (mLeftImageFilePathTemplate != "") {
            int indexEntryPoint = mLeftImageFilePathTemplate.find("%d");
            auto imagePath = mLeftImageFilePathTemplate;
            imagePath.replace(indexEntryPoint, 2, std::to_string(mFrameCounter));
            mLeft = imread(imagePath);
        }

        if (mRightImageFilePathTemplate != "") {
            int indexEntryPoint = mRightImageFilePathTemplate.find("%d");
            auto imagePath = mRightImageFilePathTemplate;
            imagePath.replace(indexEntryPoint, 2, std::to_string(mFrameCounter));
            mRight = imread(imagePath.substr(0, imagePath.size() - 1));
        }

        if(mDepthImageFilePathTemplate != ""){
            int indexEntryPoint = mDepthImageFilePathTemplate.find("%d");
            auto imagePath = mDepthImageFilePathTemplate;
            imagePath.replace(indexEntryPoint, 2, std::to_string(mFrameCounter));
            mDepth = imread(imagePath, CV_LOAD_IMAGE_UNCHANGED);
        }


		return true;
	}

    //---------------------------------------------------------------------------------------------------------------------
    void StereoCameraVirtual::depthToPointcloud(Mat & _depth, PointCloud<PointXYZ>& _cloud) {
        // Fake parameters
        int cx = mMatrixRight.at<float>(0,2);
        int cy = mMatrixRight.at<float>(1,2);;
        double fx = mMatrixRight.at<float>(0,0);
        double fy = mMatrixRight.at<float>(1,1);

        for (int i = 0; i < _depth.rows; i++) {
            for (int j = 0; j < _depth.cols; j++) {
                double z = double(_depth.at<unsigned short>(i*_depth.cols + j)) * mDispToDepth;
                if (!z)
                    continue;
                double x = double(j - cx)*z / fx;
                double y = double(i - cy)*z / fy;
                _cloud.push_back(PointXYZ(x, y, z));
            }
        }
        _cloud.is_dense = false; // 666 TODO: cant set to true if wrong points are set to NaN.
        _cloud.width = mLeft.cols;
        _cloud.height = mLeft.rows;

    }

    //---------------------------------------------------------------------------------------------------------------------
    void StereoCameraVirtual::depthToPointcloud(Mat & _depth, PointCloud<PointXYZRGB>& _cloud) {
        // Fake parameters
        int cx = mMatrixRight.at<float>(0,2);
        int cy = mMatrixRight.at<float>(1,2);;
        double fx = mMatrixRight.at<float>(0,0);
        double fy = mMatrixRight.at<float>(1,1);

        for (int i = 0; i < _depth.rows; i++) {
            for (int j = 0; j < _depth.cols; j++) {
                double z = double(_depth.at<unsigned short>(i*_depth.cols + j)) * mDispToDepth;
                PointXYZRGB p;
                if (z){
                    double x = double(j - cx)*z / fx;
                    double y = double(i - cy)*z / fy;

                    p.x = x;
                    p.y = y;
                    p.z = z;
                    p.r = mLeft.at<Vec3b>(i,j)[2];
                    p.g = mLeft.at<Vec3b>(i,j)[1];
                    p.b = mLeft.at<Vec3b>(i,j)[0];
                }else{
                    p.x = NAN;
                    p.y = NAN;
                    p.z = NAN;
                    p.r = 0;
                    p.g = 0;
                    p.b = 0;
                }

                _cloud.push_back(p);
            }
        }

        _cloud.is_dense = false; // 666 TODO: cant set to true if wrong points are set to NaN.
        _cloud.width = mLeft.cols;
        _cloud.height = mLeft.rows;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraVirtual::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point){
        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value = mDepth.at<uint16_t>(_pixel.y, _pixel.x);
        float depth_in_meters = depth_value * mDispToDepth;
        // Set invalid pixels with a depth value of zero, which is used to indicate no data
        if (depth_value == 0) {
            return false;
        }
        else {
            // 666 Assuming that it is undistorted which is for intel real sense F200 and depth is in color CS...
            _point.x = (_pixel.x - mMatrixLeft.at<float>(0,2))/mMatrixLeft.at<float>(0,0)*depth_in_meters;
            _point.y = (_pixel.y - mMatrixLeft.at<float>(1,2))/mMatrixLeft.at<float>(1,1)*depth_in_meters;
            _point.z = depth_in_meters;
            return true;
        }
    }
}	//	namespace rgbd
