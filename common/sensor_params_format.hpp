#pragma once
#include "switchboard.hpp"


// @todo Add Eigen matrices for camera intrinsics, extrinsics

// Camera params
struct cam_params : public switchboard::event {
    std::string camera_id;
    Eigen::Matrix<double, 3, 3> intrinsics;
    //
    // [ Fx s x0]
    // [ 0 Fy y0]
    // [ 0  0  1]
    //
    Eigen::Matrix<double, 3, 4> extrinsics;

    int frame_rate;
    int resolution[2];
    std::string camera_model;
    std::string distortion_model;
    float distortion_coefficients[4];


    //Based on https://github.com/ILLIXR/Kimera-VIO/blob/master/include/kimera-vio/frontend/CameraParams.h
    // we need to provide the following information at minimum to the SLAM plugin
    
    // Id of the camera
    CameraId camera_id_;

    // Camera model: pinhole, etc
    std::string camera_model_;

    // fu, fv, cu, cv
    Intrinsics intrinsics_;
    // OpenCV structures: needed to compute the undistortion map.
    // 3x3 camera matrix K (last row is {0,0,1})
    cv::Mat K_;

    // Sensor extrinsics wrt body-frame
    gtsam::Pose3 body_Pose_cam_;

    // Image info.
    double frame_rate_;
    cv::Size image_size_;

    // GTSAM structures to calibrate points:
    // Contains intrinsics and distortion parameters.
    gtsam::Cal3DS2 calibration_;

    // Distortion parameters
    DistortionModel distortion_model_;
    std::vector<double> distortion_coeff_;
    cv::Mat distortion_coeff_mat_;

    // TODO(Toni): Legacy, pls remove.
    cv::Mat undistort_rectify_map_x_;
    cv::Mat undistort_rectify_map_y_;

    // TODO(Toni): why do we have rectification stuff here? This should be
    // for stereo cameras only...
    // Rotation resulting from rectification.
    cv::Mat R_rectify_;

    // Camera matrix after rectification.
    cv::Mat P_;

    //! List of Cameras which share field of view with this one: i.e. stereo.
    std::vector<CameraId> is_stereo_with_camera_ids_;
}

// @todo Add floats for IMU parameters
