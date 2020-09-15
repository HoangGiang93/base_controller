#! /usr/bin/env python
import rospy

import actionlib

from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult, FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist, Pose, Quaternion
from sensor_msgs.msg import JointState

class BaseControl(object):
  # create messages that are used to publish feedback/result
  _feedback = FollowJointTrajectoryActionFeedback()
  _result = FollowJointTrajectoryActionResult()

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=10)

    joint_states = rospy.wait_for_message('/joint_states', JointState)
    try:
      self.odom_x_joint_index = joint_states.name.index(odom_x_joint)
      self.odom_y_joint_index = joint_states.name.index(odom_y_joint)
      self.odom_z_joint_index = joint_states.name.index(odom_z_joint)
      self.joint_names = [odom_x_joint, odom_y_joint, odom_z_joint]
      rospy.loginfo("base_control found odom joints")
    except ValueError as e:
      rospy.logwarn("base_control couldn't find odom joints in joint states!")
      return
    
    self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=10)

    # create state publisher
    self.state_pub = rospy.Publisher('{}/state'.format(name_space), JointTrajectoryControllerState, queue_size=10)

    # create the action server
    self._as = actionlib.SimpleActionServer('{}/follow_joint_trajectory'.format(name_space), FollowJointTrajectoryAction, self.goal_callback, False)
    self._as.start()

  def joint_states_callback(self, joint_states):
    
    try:
      self.current_positions = [joint_states.position[self.odom_x_joint_index], joint_states.position[self.odom_y_joint_index], joint_states.position[self.odom_z_joint_index]]
      # rospy.loginfo("base_control found odom joint positions")
    except IndexError as e:
      rospy.logwarn("base_control couldn't find enough odom joint positions in joint states!")
      return

    try:
      self.current_velocities = [joint_states.velocity[self.odom_x_joint_index], joint_states.velocity[self.odom_y_joint_index], joint_states.velocity[self.odom_z_joint_index]]
      # rospy.loginfo("base_control found odom joint velocites")
    except IndexError as e:
      rospy.logwarn("base_control couldn't find enough odom joint velocities in joint states!")
      return
    
    state = JointTrajectoryControllerState()
    state.joint_names = self.joint_names
    self.state_pub.publish(state)

  def goal_callback(self, goal):

    # helper variables
    rate = rospy.Rate(20)
    success = True
    t = 0
    try:
      goal_odom_x_joint_index = goal.trajectory.joint_names.index(odom_x_joint)
      goal_odom_y_joint_index = goal.trajectory.joint_names.index(odom_y_joint)
      goal_odom_z_joint_index = goal.trajectory.joint_names.index(odom_z_joint)
    except:
      rospy.loginfo("base_control aborted current goal")
      self._as.set_aborted()
      return

    cmd_vel = Twist()
    while True:
      if self._as.is_preempt_requested():
        rospy.loginfo("The goal has been preempted")
        # the following line sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False
        break

      if len(goal.trajectory.points) == t:
        break
      
      # error = sum([abs(self.current_positions[i] - goal.trajectory.points[t].positions[i]) for i in range(3)])
      error = abs(self.current_positions[0] - goal.trajectory.points[t].positions[0])
      print("At", t, "x_ist = ", self.current_positions[0], "x_soll = ", goal.trajectory.points[t].positions[0])
      print("At", t, "error = ", error)
    
      self._feedback.feedback.joint_names = self.joint_names
      self._feedback.feedback.actual.positions = self.current_positions
      self._feedback.feedback.actual.velocities = self.current_velocities
      self._feedback.feedback.actual.time_from_start.secs += 0.05
      
      # publish the feedback
      self._as.publish_feedback(self._feedback.feedback)

      # publish the velocity
      cmd_vel.linear.x = goal.trajectory.points[t].velocities[goal_odom_x_joint_index]
      cmd_vel.linear.y = goal.trajectory.points[t].velocities[goal_odom_y_joint_index]
      cmd_vel.angular.z = goal.trajectory.points[t].velocities[goal_odom_z_joint_index]
      self.cmd_vel_pub.publish(cmd_vel)
      t += 1
      rate.sleep()

    cmd_vel.linear.x = 0
    cmd_vel.linear.y = 0
    cmd_vel.angular.z = 0
    self.cmd_vel_pub.publish(cmd_vel)

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