#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************

PKG = "pr2_move_base"

import roslib; roslib.load_manifest(PKG) 

import rospy
import actionlib
from pr2_msgs.msg import LaserTrajCmd
from pr2_msgs.srv import SetLaserTrajCmd
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
import dynamic_reconfigure.client
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

def feedback_cb(feedback):
    server.publish_feedback(feedback)

def set_tilt_profile(position, time_from_start):
    cmd = LaserTrajCmd()
    cmd.profile = 'blended_linear'
    cmd.position = position
    cmd.time_from_start = [rospy.Time.from_sec(x) for x in time_from_start]
    cmd.max_velocity = 10
    cmd.max_acceleration = 30
    try:
        tilt_profile_client.call(cmd)
    except rospy.ServiceException, e:
        rospy.logerr("Couldn't set the profile on the laser. Exception %s" % e)
        return False
    return True

def configure_head():
    head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
    head_client.wait_for_server()
    point_head_goal = PointHeadGoal()
    point_head_goal.target.header.frame_id = 'base_link'
    point_head_goal.target.point.x = 3.0
    point_head_goal.target.point.y = 0.0
    point_head_goal.target.point.z = 1.0
    point_head_goal.pointing_frame = 'head_pan_link'
    point_head_goal.pointing_axis.x = 1
    point_head_goal.pointing_axis.y = 0
    point_head_goal.pointing_axis.z = 0

    head_client.send_goal(point_head_goal)
    head_client.wait_for_result(rospy.Duration(5.0))


def configure_laser():
    #TODO: remove hack to get things working in gazebo
    try: 
        rospy.wait_for_service('tilt_hokuyo_node/set_parameters', 1.0)
    except rospy.exceptions.ROSException, e:
        rospy.logerr("Couldn't set parameters %s" % e)
        return
    #end TODO

    client = dynamic_reconfigure.client.Client('tilt_hokuyo_node')
    global old_config
    old_config = client.get_configuration(2.0)
    new_config = {'skip': 0, 'intensity': 0, 'min_ang': -1.57, 'max_ang': 1.57, 'calibrate_time': 1, 'cluster': 1, 'time_offset': 0.0} 
    rospy.loginfo('Setting laser to the navigation configuration: %s' % new_config)
    client.update_configuration(new_config)

def restore_laser():
    #TODO: remove hack to get things working in gazebo
    try: 
        rospy.wait_for_service('tilt_hokuyo_node/set_parameters', 1.0)
    except rospy.exceptions.ROSException, e:
        rospy.logerr("Couldn't set parameters %s" % e)
        return
    #end TODO

    rospy.loginfo('Setting laser back to this configuration: %s' % old_config)
    client = dynamic_reconfigure.client.Client('tilt_hokuyo_node')
    client.update_configuration(old_config)

def on_shutdown():
    restore_laser()
    set_tilt_profile([0.0, 0.0], [0.0, 1.0])

def execute_cb(goal):
    rospy.loginfo("Received a goal")
    if not set_tilt_profile([1.05,  -.7, 1.05], [0.0, 1.8, 2.0125 + .3]):
        server.set_aborted(MoveBaseResult(), "Couldn't set the profile on the laser")
        return

    configure_laser()
    configure_head()

    move_base_client.send_goal(goal, None, None, feedback_cb)

    while not move_base_client.wait_for_result(rospy.Duration(0.1)):
        if server.is_preempt_requested():
            if not server.is_new_goal_available():
                move_base_client.cancel_goal()

            server.set_preempted(MoveBaseResult(), "Got preempted by a new goal")
            return


    terminal_state = move_base_client.get_state()
    result = move_base_client.get_result()
    if terminal_state == GoalStatus.PREEMPTED:
        server.set_preempted(result)
    elif terminal_state == GoalStatus.SUCCEEDED:
        server.set_succeeded(result)
    elif terminal_state == GoalStatus.ABORTED:
        server.set_aborted(result)
    else:
        server.set_aborted(result, "Unknown result from move_base")


if __name__ == '__main__':
    name = 'pr2_move_base'
    rospy.init_node(name)

    #create the action client to move_base
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()
    move_base_goal = MoveBaseGoal()

    #create a client to the tilt laser
    rospy.wait_for_service('laser_tilt_controller/set_traj_cmd')
    tilt_profile_client = rospy.ServiceProxy('laser_tilt_controller/set_traj_cmd', SetLaserTrajCmd)

    server = actionlib.SimpleActionServer(name, MoveBaseAction, execute_cb)
    rospy.loginfo('%s: Action server running', name)

    global old_config
    old_config = {}

    #we'll set the tilt profile and configure the laser by default when we start up
    set_tilt_profile([1.05,  -.7, 1.05], [0.0, 1.8, 2.0125 + .3])
    configure_laser()

    rospy.on_shutdown(on_shutdown)

    rospy.spin()
