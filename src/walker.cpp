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
 * @file walker.cpp
 *
 * @brief This is the source code that creates a ROS node to make the turtlebot
 * behave as if it were a roomba.
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "Behaviour.cpp"

#include <algorithm>
#include <memory>
#include <vector>
#include <math.h>

std::shared_ptr<Behaviour> behaviour;

bool isNan(float i) {
  return std::isnan(i);
}
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  std::vector<float> ranges = msg->ranges;
  // Replace all nans with max range to get the actual minimum
  std::replace_if(ranges.begin(), ranges.end(), isNan, msg->range_max);

  float minRange = *std::min_element(ranges.begin(), ranges.end());
  ROS_INFO_STREAM("The closest object is " << minRange << "(m) away.");
  behaviour->updateMinDist(minRange);
}

int main(int argc, char **argv) {

  // Initialize the walker node.
  ros::init(argc, argv, "walker");
  // Initialize the shared pointer to the behaviour class that will be used to
  // process the point cloud call backs.
  behaviour.reset(new Behaviour);

  // Create a node handle
  ros::NodeHandle n;

  // Publish on the topic required to move turtlebot
  // This will be remmapped in the launch file.
  auto pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  auto sub = n.subscribe("/scan", 1000, laserScanCallback);

  // Publish at 10 Hz.
  ros::Rate loop_rate(10.0);

  while (ros::ok()) {

    // Publish the velocity as determined by the behavior class.
    geometry_msgs::Twist msg;  // Call to behavior class here.
    std::pair<double, double> output = behaviour->computeVelocities();
    msg.linear.x = output.first;
    msg.angular.z = output.second;

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
