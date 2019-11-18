/*
 * @copyright Copyright 2019 <Corbyn Yhap>
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
 * @author Corbyn Yhap
 *
 * @file behaviour.cpp
 *
 * @brief This computes linear and angular velocities to operate a turtlebot as
 * if it were driving like a roomba.
 *
 */
#include "Behaviour.hpp"

// Note the constructor assumes there is an object in front of it until it gets
// a reading telling it otherwise. Resulting in turning at the very beginning
// of the simulation.
Behaviour::Behaviour(const double &aColDist, const double &aLinVel,
                     const double &angVel)
    :
    clearAhead(false),
    collisionDist(aColDist),
    maxLinVel(aLinVel),
    maxAngVel(angVel) {
}

Behaviour::~Behaviour() {
}

void Behaviour::updateMinDist(float aDist) {
  clearAhead = true;
  if (aDist <= collisionDist) {
    clearAhead = false;
  }
}

std::pair<double, double> Behaviour::computeVelocities() {
  double linearVel;
  double angularVel;
  if (clearAhead) {
    linearVel = maxLinVel;
    angularVel = 0;
  } else {
    linearVel = 0;
    angularVel = maxAngVel;
  }
  return std::make_pair(linearVel, angularVel);
}
