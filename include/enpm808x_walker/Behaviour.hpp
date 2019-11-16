/* Copyright (c) 2019, Corbyn Yhap
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file Behaviour.hpp
 *
 * @brief This computes linear and angular velocities to operate a turtlebot as
 * if it were driving like a roomba.
 *
 * @author Corbyn Yhap
 */
#pragma once

#include <vector>
#include <utility>

#include "sensor_msgs/PointCloud2.h"


class Behaviour {
 public:
  /**

   * @brief Constructor for the Behaviour Class

   * @param None.

   * @return None.

   */
  Behaviour();

  /**

   * @brief Destructor for the Behaviour Class

   * @param None.

   * @return None.

   */
  ~Behaviour();

  /**

   * @brief Uses the turtlebot sensor data to determine whether or not somthing
   * is in front of the robot. Sets the clearAhead variable accordingly.

   * @param sensor_msgs::PointCloud2ConstPtr The Point cloud data that will be
   * used to determine if the robot is in danger of colliding.

   * @return None.

   */
  void updateInfo(sensor_msgs::PointCloud2ConstPtr);

  /**

   * @brief Checks the clear ahead boolean and updates the velocities
   * accordingly. (Straight if clear ahead. Turning if obstacle in front.)

   * @param None

   * @return std::pair<double, double> The linear and angular velocities
   * respectively.

   */
  std::pair<double, double> computeVelocities();

 private:
  double linearVel;
  double angularVel;
  bool clearAhead;
};
