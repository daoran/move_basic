#!/usr/bin/env python

"""
Copyright (c) 2020, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of display_node nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Example client program for sending multiple move_basic commands
"""

import rospy
import getopt
import sys
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Controller:
    """
    Controller for moving the robot through predefined waypoints.
    Supports different patterns (line, box, octagon, figure-8).
    """

    # Predefined waypoints (x, y, yaw, comment)
    waypoints = {
        "line": [
            [0.00, 0.00, 0.000, "MOVE: Leg A"],
            [0.50, 0.00, 0.000, "MOVE: Leg B"]
        ],
        "box": [
            [0.00, 0.00, 0.000, "MOVE: Leg A"],
            [0.50, 0.00, 1.570, "MOVE: Leg B"],
            [0.50, 0.50, 3.140, "MOVE: Leg C"],
            [0.00, 0.50, -1.570, "MOVE: Leg D"]
        ],
        "octagon": [
            [0.40, 0.30, 0.000, "MOVE: Leg A"],
            [0.60, 0.10, -0.785, "MOVE: Leg B"],
            [0.60, -0.10, -1.571, "MOVE: Leg C"],
            [0.40, -0.30, -2.356, "MOVE: Leg D"],
            [0.20, -0.30, -3.141, "MOVE: Leg E"],
            [0.00, -0.10, 2.356, "MOVE: Leg F"],
            [0.00, 0.10, 1.571, "MOVE: Leg G"],
            [0.20, 0.30, 0.785, "MOVE: Leg H"]
        ],
        "figure8": [
            # First Octagon
            [0.40, 0.30, 0.000, "MOVE: Leg A"],
            [0.60, 0.10, -0.785, "MOVE: Leg B"],
            [0.60, -0.10, -1.571, "MOVE: Leg C"],
            [0.40, -0.30, -2.356, "MOVE: Leg D"],
            [0.20, -0.30, -3.141, "MOVE: Leg E"],
            [0.00, -0.10, 2.356, "MOVE: Leg F"],
            [0.00, 0.10, 1.571, "MOVE: Leg G"],
            [0.20, 0.30, 0.785, "MOVE: Leg H"],
            # Second Octagon
            [-0.20, 0.30, 2.356, "MOVE: Leg I"],
            [-0.40, 0.30, 3.141, "MOVE: Leg J"],
            [-0.60, 0.10, -2.356, "MOVE: Leg K"],
            [-0.60, -0.10, -1.571, "MOVE: Leg L"],
            [-0.40, -0.30, -0.785, "MOVE: Leg M"],
            [-0.20, -0.30, 0.000, "MOVE: Leg N"],
            [0.00, -0.10, 0.785, "MOVE: Leg O"],
            [0.00, 0.10, 1.571, "MOVE: Leg P"]
        ]
    }

    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Default values
        self.wait_at_each_vertex = True
        self.scale_x = 1.0
        self.scale_y = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.waypoint_name = "line"
        self.waypoint_list = self.waypoints[self.waypoint_name]

        # Parse command-line arguments
        self.parse_arguments()

        rospy.loginfo(f"Selected waypoints: {self.waypoint_name}")
        rospy.loginfo(f"Scale: ({self.scale_x}, {self.scale_y}), Offset: ({self.offset_x}, {self.offset_y})")
        rospy.loginfo(f"Total waypoints: {len(self.waypoint_list)}")

    def parse_arguments(self):
        """Parse command-line arguments to configure the waypoint navigation."""
        try:
            opts, _ = getopt.getopt(sys.argv[1:], "hcw:s:x:y:", 
                ["help", "continue", "waypoints=", "scale=", "offsetX=", "offsetY="])
        except getopt.GetoptError:
            rospy.logerr("Error parsing arguments")
            self.print_usage()
            sys.exit(2)

        for o, a in opts:
            if o in ("-h", "--help"):
                self.print_usage()
                sys.exit(0)
            elif o in ("-c", "--continue"):
                self.wait_at_each_vertex = False
            elif o in ("-w", "--waypoints"):
                if a in self.waypoints:
                    self.waypoint_name = a
                    self.waypoint_list = self.waypoints[a]
                else:
                    rospy.logerr("Invalid waypoint list name")
                    sys.exit(2)
            elif o in ("-s", "--scale"):
                self.scale_x = self.scale_y = float(a)
            elif o in ("-x", "--offsetX"):
                self.offset_x = float(a)
            elif o in ("-y", "--offsetY"):
                self.offset_y = float(a)

    def publish_move_base_goal(self, x, y, yaw, comment):
        """Send a goal to the move_base action server."""
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, \
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = q

        rospy.loginfo(f"Publishing goal: {comment} | X: {x:.2f}, Y: {y:.2f}, Yaw: {yaw:.2f}")
        client.send_goal(goal)
        client.wait_for_result()

        if client.get_result():
            rospy.loginfo(f"Waypoint reached: {comment}")
        else:
            rospy.logwarn(f"Failed to reach waypoint: {comment}")

    def run(self):
        """Main loop for waypoint navigation."""
        rospy.loginfo("Starting waypoint navigation...")

        while not rospy.is_shutdown():
            for x, y, yaw, comment in self.waypoint_list:
                x = (x * self.scale_x) + self.offset_x
                y = (y * self.scale_y) + self.offset_y

                self.publish_move_base_goal(x, y, yaw, comment)

                if self.wait_at_each_vertex:
                    rospy.loginfo("Waiting at waypoint...")
                    rospy.sleep(2)  # Replaces raw_input for better ROS integration

    def shutdown(self):
        """Stops the robot on shutdown."""
        rospy.loginfo("Shutting down waypoint controller.")

    @staticmethod
    def print_usage():
        print("Usage:")
        print("  -h, --help       Show this help message")
        print("  -c, --continue   Continuous operation (no waiting at each vertex)")
        print("  -w, --waypoints  Select a predefined waypoint list (line, box, octagon, figure8)")
        print("  -s, --scale      Scale the pattern size")
        print("  -x, --offsetX    Offset in X direction")
        print("  -y, --offsetY    Offset in Y direction")

if __name__ == "__main__":
    controller = Controller()
    controller.run()
