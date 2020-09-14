#! /usr/bin/env python
import rospy

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult, FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist, Pose, Quaternion
from sensor_msgs.msg import JointState

class BaseControl(object):
  # create messages that are used to publish feedback/result
  _feedback = FollowJointTrajectoryActionFeedback()
  _result = FollowJointTrajectoryActionResult()

  def __init__(self):
    self.cmd_vel_sub = rospy.Publisher('~cmd_vel', Twist, queue_size=10)

    joint_states = rospy.wait_for_message('/joint_states', JointState)
    try:
      self.odom_x_joint_index = joint_states.name.index(odom_x_joint)
      self.odom_y_joint_index = joint_states.name.index(odom_y_joint)
      self.odom_z_joint_index = joint_states.name.index(odom_z_joint)
      rospy.loginfo("base_control found odom joints")
    except ValueError as e:
      rospy.logwarn("base_control couldn't find odom joints in joint states!")
      return
    
    self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=10)

    # creates the action server
    self._as = actionlib.SimpleActionServer("base_control_server", FollowJointTrajectoryAction, self.goal_callback, False)
    self._as.start()

  def joint_states_callback(self, joint_states):
    
    try:
      self.current_positions = [joint_states.position[self.odom_x_joint_index], joint_states.position[self.odom_y_joint_index], joint_states.position[self.odom_z_joint_index]]
      rospy.loginfo("base_control found odom joint positions")
    except IndexError as e:
      rospy.logwarn("base_control couldn't find enough odom joint positions in joint states!")
      return

    try:
      current_velocities = [joint_states.velocity[self.odom_x_joint_index], joint_states.velocity[self.odom_y_joint_index], joint_states.velocity[self.odom_z_joint_index]]
      rospy.loginfo("base_control found odom joint velocites")
    except IndexError as e:
      rospy.logwarn("base_control couldn't find enough odom joint velocities in joint states!")
      return
    

  def goal_callback(self, goal):

    # helper variables
    rate = rospy.Rate(1)
    success = True

    try:
      goal_odom_x_joint_index = goal.trajectory.joint_names.index(odom_x_joint)
      goal_odom_y_joint_index = goal.trajectory.joint_names.index(odom_y_joint)
      goal_odom_z_joint_index = goal.trajectory.joint_names.index(odom_z_joint)
    except:
      rospy.loginfo("base_control aborted current goal")
      self._as.set_aborted()
      return

    for i in range(5):
      if self._as.is_preempt_requested():
        rospy.loginfo("The goal has been preempted")
        # the following line sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False
        break

      self._feedback.feedback.joint_names = ["a", "b", "c"]
      self._feedback.feedback.actual.positions.append(1)
      self._feedback.feedback.actual.velocities.append(2)
      self._feedback.feedback.actual.accelerations.append(3)
      self._feedback.feedback.actual.time_from_start.secs += 1
      
      # publish the feedback
      self._as.publish_feedback(self._feedback.feedback)
      rate.sleep()

    if success:
      rospy.loginfo("The goal has been reached")
      self._as.set_succeeded()

if __name__ == '__main__':
  rospy.init_node("base_control")
  name_space = rospy.get_param('~name_space')
  odom_x_joint = rospy.get_param('{}/odom_x_joint'.format(name_space))
  odom_y_joint = rospy.get_param('{}/odom_y_joint'.format(name_space))
  odom_z_joint = rospy.get_param('{}/odom_z_joint'.format(name_space))

  # publish info to the console for the user
  rospy.loginfo("base_control starts")

  # start the base control
  BaseControl()

  # keep it running
  rospy.spin()