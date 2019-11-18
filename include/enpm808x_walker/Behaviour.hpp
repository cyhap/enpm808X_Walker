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

#include <utility>

class Behaviour {
 public:
  /**

   * @brief Constructor for the Behaviour Class

   * @param const double &. The Collision Distance Used

   * @param const double &. The Linear Velocity Used

   * @param const double &. The Angular Velocity Used

   * @return None.

   */
  Behaviour(const double &aColDist = 0.65, const double &aLinVel = 0.25,
            const double &angVel = 1);

  /**

   * @brief Destructor for the Behaviour Class

   * @param None.

   * @return None.

   */
  ~Behaviour();

  /**

   * @brief Uses the turtlebot sensor data to determine whether or not something
   * is in front of the robot. Sets the clearAhead variable accordingly.

   * @param float the minimum distance observed

   * @return None.

   */
  void updateMinDist(float);

  /**

   * @brief Checks the clear ahead boolean and updates the velocities
   * accordingly. (Straight if clear ahead. Turning if obstacle in front.)

   * @param None

   * @return std::pair<double, double> The linear and angular velocities
   * respectively.

   */
  std::pair<double, double> computeVelocities();

  /**

   * @brief Returns whether or not the robot can move forward. This function was
   * added so that unit testing could take place on the updateMinDist function.

   * @param None

   * @return bool. Whether or not there is something in range that is less than
   * the collisionDist member variable.

   */
  bool getClearAhead();

 private:
  // This is the boolean indicating whether the robot can move forward.
  bool clearAhead;
  // This is the minimum distance reading allowed before turning starts.
  float collisionDist;
  // This is linear velocity when path is clear
  double maxLinVel;
  // This is the angular velocity when path is not clear.
  double maxAngVel;
};
