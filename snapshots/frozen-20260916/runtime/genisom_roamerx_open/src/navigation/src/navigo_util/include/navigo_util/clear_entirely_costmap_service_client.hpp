// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAVIGO_UTIL__CLEAR_ENTIRELY_COSTMAP_SERVICE_CLIENT_HPP_
#define NAVIGO_UTIL__CLEAR_ENTIRELY_COSTMAP_SERVICE_CLIENT_HPP_

#include <string>
#include "navigo_util/service_client.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace navigo_util
{
/**
 * @class navigo_util::ClearEntirelyCostmapServiceClient
 * @brief A service client to clear costmaps entirely
 */
class ClearEntirelyCostmapServiceClient
  : public navigo_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  /**
* @brief A constructor for navigo_util::ClearEntirelyCostmapServiceClient
   */
  explicit ClearEntirelyCostmapServiceClient(const std::string & service_name)
  : navigo_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>(service_name)
  {
  }

  using clearEntirelyCostmapServiceRequest =
    navigo_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>::RequestType;
  using clearEntirelyCostmapServiceResponse =
    navigo_util::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>::ResponseType;
};

}  // namespace navigo_util

#endif  // NAVIGO_UTIL__CLEAR_ENTIRELY_COSTMAP_SERVICE_CLIENT_HPP_
