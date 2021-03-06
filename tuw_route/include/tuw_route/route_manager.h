/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George Todoran <todorangrg@gmail.com>   *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef ROUTE_MANAGER_H
#define ROUTE_MANAGER_H

#include <memory>

#include <tuw_route/route.h>

namespace tuw
{
class RouteManager;
using RouteManagerPtr = std::shared_ptr<RouteManager>;
using RouteManagerConstPtr = std::shared_ptr<const RouteManager>;

class RouteManager : public Route
{
public:
  // special class member functions
public:
  RouteManager();

public:
  virtual ~RouteManager() = default;

public:
  RouteManager(const RouteManager&) = default;

public:
  RouteManager& operator=(const RouteManager&) = default;

public:
  RouteManager(RouteManager&&) = default;

public:
  RouteManager& operator=(RouteManager&&) = default;

public:
  void loadRoute(const std::vector<Pose2D>& _pointsSeq);

public:
  void update(const Pose2D& _agentPose, const bool& _keepLast = false);

public:
  void computeWaypointsDistanceToGoal();

public:
  void pauseCurrentRoute();

public:
  void resumeCurrentRoute();

public:
  Pose2D agentPose();

public:
  double route_waypoint_sample_dist_;

public:
  double route_max_deviation_;

public:
  double waypoint_active_arc_len_;

public:
  double visited_waypoint_min_dangle_;

  /** updates the state of all the loaded waypoints */
private:
  void updateWaypoints(const bool& _keepLast = false);

private:
  Pose2D agent_pose_;
};
}
#endif  // ROUTE_MANAGER_H
