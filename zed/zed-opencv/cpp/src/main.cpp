///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 ** This sample demonstrates how to use the ZED SDK with OpenCV.                              **
 ** Depth and images are captured with the ZED SDK, converted to OpenCV format and displayed. **
 ***********************************************************************************************/

// ZED includes
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <SaveDepth.hpp>
#include <thread>

#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"

using namespace sl;
using namespace std;
using namespace ILLIXR;

cv::Mat slMat2cvMat(Mat& input);

void printHelp();

class camerastart {
public:
  camerastart() { }

  std::shared_ptr<Camera> start_camera() {
    std::shared_ptr<Camera> zedm = std::make_shared<Camera>();

    InitParameters init_params;
    // Cam Setup
    init_params.camera_resolution = RESOLUTION::VGA;
    init_params.depth_mode = DEPTH_MODE::ULTRA;
    init_params.coordinate_units = UNIT::METER;

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
    // Cam setup
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;
    image_size = zedm->getCameraInformation().camera_resolution;
    //imageL_zed = Mat(image_size.width, image_size.height, MAT_TYPE::U8_C4);
    //imageR_zed = Mat(image_size.width, image_size.height, MAT_TYPE::U8_C4);
    imageL_zed.alloc(image_size.width, image_size.height, MAT_TYPE::U8_C4, MEM::CPU);
    imageR_zed.alloc(image_size.width, image_size.height, MAT_TYPE::U8_C4, MEM::CPU);
    imageL_ocv = slMat2cvMat(imageL_zed);
    imageR_ocv = slMat2cvMat(imageR_zed);
  }

  // using std::atomic b/c data is passed between threads
  std::atomic<cv::Mat*> img0;
  std::atomic<cv::Mat*> img1;

protected:
  virtual void _p_one_iteration() override {
    if (zedm->grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

      std::cout << "Camera Test *******************************************************************" << std::endl;
      // Retrieve images
      zedm->retrieveImage(imageL_zed, VIEW::LEFT, MEM::CPU, image_size);
      zedm->retrieveImage(imageR_zed, VIEW::RIGHT, MEM::CPU, image_size);

      imageL_zed.updateGPUfromCPU();
      imageR_zed.updateGPUfromCPU();

      // Convert to Grayscale
      cv::cvtColor(imageL_ocv, grayL, CV_BGR2GRAY);
      cv::cvtColor(imageR_ocv, grayR, CV_BGR2GRAY);

      // Display images using cv:Mat which share sl:Mat data
      cv::imshow("cam0", grayL);
      cv::imshow("cam1", grayR);

      img0 = &grayL;
      img1 = &grayR;
    }
  }

private:
  std::shared_ptr<Camera> zedm;

  // Set runtime parameters after opening the camera
  RuntimeParameters runtime_parameters;

  Resolution image_size;

  // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
  // Only the headers and pointer to the sl::Mat are copied, not the data itself
  Mat imageL_zed;
  Mat imageR_zed;

  cv::Mat imageL_ocv;
  cv::Mat imageR_ocv;

  cv::Mat grayL;
  cv::Mat grayR;
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
        , zedm{camerastart_.start_camera()}
        , camera_thread_{zedm}
    {
      camera_thread_.start();
    }

    // deconstructor
    virtual ~zed() override {
        zedm->close();
    }

protected: // a continuous loop
    virtual void _p_one_iteration() override {
        zedm->getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);

        if (sensors_data.imu.timestamp > last_imu_ts) {
            std::cout << "IMU Test *******************************************************************" << std::endl;

            // Time as ullong (nanoseconds)
            imu_time = static_cast<ullong>(sensors_data.imu.timestamp.getNanoseconds());
            cam_time = static_cast<ullong>(zedm->getTimestamp(TIME_REFERENCE::IMAGE));

            // Time as time_point
            using time_point = std::chrono::system_clock::time_point;
            time_point uptime_timepoint{std::chrono::duration_cast<time_point::duration>(std::chrono::nanoseconds(sensors_data.imu.timestamp.getNanoseconds()))};
            std::time_t time2 = std::chrono::system_clock::to_time_t(uptime_timepoint);
            t = std::chrono::system_clock::from_time_t(time2);

            // Linear Acceleration and Angular Velocity
            Eigen::Vector3f la {sensors_data.imu.linear_acceleration.x, sensors_data.imu.linear_acceleration.y, sensors_data.imu.linear_acceleration.z};
            Eigen::Vector3f av {sensors_data.imu.angular_velocity.x, sensors_data.imu.angular_velocity.x, sensors_data.imu.angular_velocity.z};

            // std::optional<cv::Mat*> img0 = camera_thread_.img0.load();
            // std::optional<cv::Mat*> img1 = camera_thread_.img1.load();
            //
            // if (imu_time == cam_time) {
            //   img0 = camera_thread_.img0;
            //   img1 = camera_thread_.img1;
            // } else {
            //   img0 = std::nullopt;
            //   img1 = std::nullopt;
            // }
            //
            // _m_imu_cam->put(new imu_cam_type{
            //   t,
            //   av,
            //   la,
            //   img0,
            //   img1,
            //   imu_time,
            //   });

            cv::Mat* img0 = camera_thread_.img0.load();
            cv::Mat* img1 = camera_thread_.img1.load();

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

            last_imu_ts = sensors_data.imu.timestamp;
        }
    }

private:
    std::shared_ptr<Camera> zedm;
    camera_thread camera_thread_;
    camerastart camerastart_;

    switchboard* sb;
    std::unique_ptr<writer<imu_cam_type>> _m_imu_cam;

    // IMU
    SensorsData sensors_data;
    Timestamp last_imu_ts = 0;

    // Timestamps
    time_type t;
    ullong imu_time;
    ullong cam_time;
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(zed);

int main(int argc, char **argv) {
    return 0;
}

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

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}
