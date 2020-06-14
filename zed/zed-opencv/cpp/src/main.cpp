// ZED includes
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

//ILLIXR includes
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"

using namespace sl;
using namespace ILLIXR;

cv::Mat slMat2cvMat(Mat& input);

class camerastart {
public:
  camerastart() { }

  std::shared_ptr<Camera> start_camera() {
    std::shared_ptr<Camera> zedm = std::make_shared<Camera>();

    // Cam setup
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD720;
    init_params.depth_mode = DEPTH_MODE::ULTRA;
    init_params.coordinate_units = UNIT::METER;
    init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Coordinate system used in ROS

    // Open the camera
    ERROR_CODE err = zedm->open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zedm->close();
    }

    return zedm;
  }
};

class camera_thread : public threadloop {
public:
  camera_thread(std::shared_ptr<Camera> zedm_) : zedm{zedm_}
  {
    // Image setup
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;
    image_size = zedm->getCameraInformation().camera_resolution;

    imageL_zed.alloc(image_size.width, image_size.height, MAT_TYPE::U8_C4, MEM::CPU);
    imageR_zed.alloc(image_size.width, image_size.height, MAT_TYPE::U8_C4, MEM::CPU);
    depth_image_zed.alloc(image_size.width, image_size.height, MAT_TYPE::U8_C4, MEM::CPU);

    imageL_ocv = slMat2cvMat(imageL_zed);
    imageR_ocv = slMat2cvMat(imageR_zed);
    depth_image_ocv = slMat2cvMat(depth_image_zed);
  }

  // for imu-cam
  std::atomic<cv::Mat*> img0;
  std::atomic<cv::Mat*> img1;
  // for reconstruction
  std::atomic<cv::Mat*> rgb;
  std::atomic<cv::Mat*> depth;

protected:
  virtual void _p_one_iteration() override {
    if (zedm->grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

      // Retrieve images
      zedm->retrieveImage(imageL_zed, VIEW::LEFT, MEM::CPU, image_size);
      zedm->retrieveImage(imageR_zed, VIEW::RIGHT, MEM::CPU, image_size);
      zedm->retrieveImage(depth_image_zed, VIEW::DEPTH, MEM::CPU, image_size);

      // Convert to Grayscale
      cv::cvtColor(imageL_ocv, grayL, CV_BGR2GRAY);
      cv::cvtColor(imageR_ocv, grayR, CV_BGR2GRAY);

      img0 = &grayL;
      img1 = &grayR;

      rgb = &imageL_ocv;
      depth = &depth_image_ocv;
    }
  }

private:
  std::shared_ptr<Camera> zedm;

  RuntimeParameters runtime_parameters;
  Resolution image_size;

  // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
  // Only the headers and pointer to the sl::Mat are copied, not the data itself
  Mat imageL_zed;
  Mat imageR_zed;
  Mat depth_image_zed;

  cv::Mat imageL_ocv;
  cv::Mat imageR_ocv;
  cv::Mat grayL;
  cv::Mat grayR;
  cv::Mat depth_image_ocv;
};

class zed : public threadloop {
public:
    // Public constructor, Spindle passes the phonebook to this
    // constructor. In turn, the constructor fills in the private
    // references to the switchboard plugs, so the plugin can read
    // the data whenever it needs to.
    zed(phonebook* pb)
        : sb{pb->lookup_impl<switchboard>()}
        , _m_imu_cam{sb->publish<imu_cam_type>("imu_cam")}
        , _m_rgb_depth{sb->publish<rgb_depth_type>("rgb_depth")}
        , zedm{camerastart_.start_camera()}
        , camera_thread_{zedm}
    {
      camera_thread_.start();
    }

    // deconstructor
    virtual ~zed() override {
        zedm->close();
    }


protected:
    // a continuous loop
    virtual void _p_one_iteration() override {
        zedm->getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);

        if (sensors_data.imu.timestamp > last_imu_ts) {

            // Time as ullong (nanoseconds)
            imu_time = static_cast<ullong>(sensors_data.imu.timestamp.getNanoseconds());

            // Time as time_point
            using time_point = std::chrono::system_clock::time_point;
            time_point uptime_timepoint{std::chrono::duration_cast<time_point::duration>(std::chrono::nanoseconds(sensors_data.imu.timestamp.getNanoseconds()))};
            std::time_t time2 = std::chrono::system_clock::to_time_t(uptime_timepoint);
            t = std::chrono::system_clock::from_time_t(time2);

            // Linear Acceleration and Angular Velocity (av converted from deg/s to rad/s)
            la = {sensors_data.imu.linear_acceleration_uncalibrated.x , sensors_data.imu.linear_acceleration_uncalibrated.y, sensors_data.imu.linear_acceleration_uncalibrated.z };
            av = {sensors_data.imu.angular_velocity_uncalibrated.x  * (M_PI/180), sensors_data.imu.angular_velocity_uncalibrated.y * (M_PI/180), sensors_data.imu.angular_velocity_uncalibrated.z * (M_PI/180)};

            // Images are swapped with NULL
            cv::Mat* img0 = camera_thread_.img0.exchange(nullptr);
            cv::Mat* img1 = camera_thread_.img1.exchange(nullptr);

            if (img0 && img1) {
              _m_imu_cam->put(new imu_cam_type{
              t,
              av,
              la,
              img0,
              img1,
              imu_time,
              });
            } else {
              _m_imu_cam->put(new imu_cam_type{
              t,
              av,
              la,
              std::nullopt,
              std::nullopt,
              imu_time,
              });
            }

            u_cam_time = zedm->getTimestamp(TIME_REFERENCE::CURRENT);
            int64_t s_cam_time = static_cast<int64_t>(u_cam_time);

            cv::Mat* r = camera_thread_.rgb.exchange(nullptr);
            cv::Mat* d = camera_thread_.depth.exchange(nullptr);

            char* rgb = new char[720 * 1280];
            short* depth = new short[720 * 1280];

            if (r && d) {
              //rgb = (char*) r->data;

              for (int i = 0; i < 720; i++) {
                for (int j = 0; j < 1280; j++) {
                  rgb = r->ptr<char*>(i)[j];
                  depth = d->ptr<short*>(i)[j];
                }
              }
              _m_rgb_depth->put(new rgb_depth_type{
              s_cam_time,
              rgb,
              depth,
              });
            }

            last_imu_ts = sensors_data.imu.timestamp;
        }
    }

private:
    std::shared_ptr<Camera> zedm;
    camera_thread camera_thread_;
    camerastart camerastart_;

    switchboard* sb;
    std::unique_ptr<writer<imu_cam_type>> _m_imu_cam;
    std::unique_ptr<writer<rgb_depth_type>> _m_rgb_depth;

    // IMU
    SensorsData sensors_data;
    Timestamp last_imu_ts = 0;
    Eigen::Vector3f la;
    Eigen::Vector3f av;

    // Timestamps
    time_type t;
    ullong imu_time;
    uint64_t u_cam_time;
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(zed);

int main(int argc, char **argv) { return 0; }

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}
