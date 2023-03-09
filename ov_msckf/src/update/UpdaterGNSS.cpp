
/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2022 Andreas Serov
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

// UGS

#include "UpdaterGNSS.h"
using namespace ov_msckf;
using namespace ov_type;
using namespace ov_core;
using namespace std;
#include <iomanip>
// UpdaterGNSS::UpdaterGNSS(VioManagerOptions &manager_options, std::shared_ptr<Propagator> prop) {

// }

// void UpdaterGNSS(const sensor_msgs::NavSatFixConstPtr &fix_msg) // UGS
// {
//   // cout<<"ubloxFix_callback received "<<endl;
//   sensor_msgs::NavSatFix navfix_;

//   navfix_.header.stamp = ros::Time::now();
//   navfix_.header.frame_id = "base_link";
//   navfix_.latitude = fix_msg->latitude;
//   navfix_.longitude = fix_msg->longitude;
//   navfix_.altitude = fix_msg->altitude;

//   gnssdata a;
//   a.lla(0) = navfix_.latitude;
//   a.lla(1) = navfix_.longitude;
//   a.lla(2) = navfix_.altitude;
//   std::cout << "THE GNSS DATA" << a.lla;
// }
void UpdaterGNSS::process_gnss(const ov_core::gnssdata &message) {

  //sensor_msgs::NavSatFix navfix_;
  //navfix_.header.stamp = ros::Time::now();
  //navfix_.header.frame_id = "base_link";
  Eigen::Vector3d gnss_values = Eigen::Vector3d(message.lat, message.log, message.alt);
  std::cout << "The GNSS data are as follows:" << std::endl<<std::fixed<<std::setprecision(9)
            << gnss_values(0) << " " <<std::fixed<<std::setprecision(9)<< gnss_values(1) << " " << gnss_values(2) << std::endl;
     

}
double UpdaterGNSS::test() { std::cout << "test" << std::endl; }
