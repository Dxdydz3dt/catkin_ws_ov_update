/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_CORE_SENSOR_DATA_H
#define OV_CORE_SENSOR_DATA_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

namespace ov_core {

/**
 * @brief Struct for  GNSS (lathitude,longitude, altitude)
 */

struct gnssdata {

  /// Timestamp of the reading
  double timestamp;
  double lat;
  double log;
  double alt;
};
/**
 * @brief Struct for a single imu measurement (time, wm, am)
 */
struct ImuData {

  /// Timestamp of the reading
  double timestamp;

  /// Gyroscope reading, angular velocity (rad/s)
  Eigen::Matrix<double, 3, 1> wm;

  /// Accelerometer reading, linear acceleration (m/s^2)
  Eigen::Matrix<double, 3, 1> am;

  /// Sort function to allow for using of STL containers
  bool operator<(const ImuData &other) const { return timestamp < other.timestamp; }
};

/**
 * @brief Struct for a collection of camera measurements.
 *
 * For each image we have a camera id and timestamp that it occured at.
 * If there are multiple cameras we will treat it as pair-wise stereo tracking.
 */
struct CameraData {

  /// Timestamp of the reading
  double timestamp;

  /// Camera ids for each of the images collected
  std::vector<int> sensor_ids;

  /// Raw image we have collected for each camera
  std::vector<cv::Mat> images;

  /// Tracking masks for each camera we have
  std::vector<cv::Mat> masks;

  /// Sort function to allow for using of STL containers
  bool operator<(const CameraData &other) const {
    if (timestamp == other.timestamp) {
      int id = *std::min_element(sensor_ids.begin(), sensor_ids.end());
      int id_other = *std::min_element(other.sensor_ids.begin(), other.sensor_ids.end());
      return id < id_other;
    } else {
      return timestamp < other.timestamp;
    }
  }
};

// OVVU
/**
 * @brief Struct for single wheel speeds measurement for 4 wheel vehicle (m/s)
 */
struct WheelSpeedsData {

  /// Timestamp of the reading
  double timestamp;

  /// Wheel speed data per wheel in meter per second
  double wheel_front_left;
  double wheel_front_right;
  double wheel_rear_left;
  double wheel_rear_right;

  /// Sort function to allow for using STL containers
  bool operator<(const WheelSpeedsData &other) const { return timestamp < other.timestamp; }
};

/**
 * @brief Struct for single AckermannDrive measurement
 *
 */
struct AckermannDriveData {

  /// Timestamp of the reading
  double timestamp;

  /// Speed reading in forward direction (m/s)
  double speed;

  /// Steering angle (rad)
  double steering_angle;

  bool operator<(const AckermannDriveData &other) const { return timestamp < other.timestamp; }
};

} // namespace ov_core

#endif // OV_CORE_SENSOR_DATA_H
