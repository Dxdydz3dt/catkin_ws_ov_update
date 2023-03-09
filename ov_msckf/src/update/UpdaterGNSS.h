/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2021 Andreas Serov
 * Copyright (C) 2021 Joachim Clemens
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

// OVVU

#ifndef OV_MSCKF_UPDATER_GNSS_H
#define OV_MSCKF_UPDATER_GNSS_H
#include "UpdaterHelper.h"
#include "UpdaterOptions.h"
#include "core/VioManagerOptions.h"
#include "sensor_msgs/NavSatFix.h" //UGS
#include "state/Propagator.h"
#include "types/Type.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"
//#include "novatel_msgs/INSPVAX.h"  //UGS
#include <boost/math/distributions/chi_squared.hpp>

namespace ov_msckf {

class UpdaterGNSS {

public:
  // Finalize the constructor by adding VioManagerOptions to this class and initializing VioManagerOptions, Propagator member
  // variable OVG

  UpdaterGNSS(VioManagerOptions &manager_options, std::shared_ptr<Propagator> prop) : _prop(prop) {
    lat1 = manager_options.lat;
    log1 = manager_options.log;
    alt1 = manager_options.alt;

    std::cout << "GNSS class is created" << std::endl << log1 << std::endl << alt1 << std::endl << lat1 << std::endl;
  }

  // GNSS
  /// Our vector of GNSS data
  // Append it to our vector
  // gnssdata.emplace_back(message);

  // Loop through and delete imu messages that are older than our requested time
  // auto it0 = gnssdata.begin();

  void process_gnss(const ov_core::gnssdata &message);

  double test();

protected:
  /// Our propagator!
  std::shared_ptr<Propagator> _prop;
  std::vector<ov_core::gnssdata> gnss_data;
  Eigen::Vector3d gnss_values;

  double lat1;
  double log1;
  double alt1;
  /// Whether to use second order yaw in the calculation of the preintegrated odometry model
  bool _use_yaw_odom_second_order = true;

  /// Whether to use second order yaw in the Jacobian derivation of the preintegrated odometry model
  bool _use_yaw_jacobi_second_order = false;
};
} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_GNSS_H
