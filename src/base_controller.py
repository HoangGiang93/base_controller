#! /usr/bin/env python
import rospy

import actionlib

from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult, FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist, Pose, Quaternion
from sensor_msgs.msg import JointState

import tf
from tf2_ros import TransformException

import math

class BaseControl(object):
  # create messages that are used to publish state/feedback/result
  _state = JointTrajectoryControllerState()
  _feedback = FollowJointTrajectoryActionFeedback()
  _result = FollowJointTrajectoryActionResult()

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher('~cmd_vel', Twist, queue_size=10)

    joint_states = rospy.wait_for_message('base/joint_states', JointState)
    try:
      self.odom_x_joint_index = joint_states.name.index(odom_x_joint)
      self.odom_y_joint_index = joint_states.name.index(odom_y_joint)
      self.odom_z_joint_index = joint_states.name.index(odom_z_joint)

      rospy.loginfo("base_controller found odom joints")
    except ValueError as e:
      rospy.logwarn("base_controller couldn't find odom joints in joint states!")
      return

    # create tf listener
    self.tf_listener = tf.TransformListener()

    # create state publisher
    self.state_pub = rospy.Publisher('{}/state'.format(name_space), JointTrajectoryControllerState, queue_size=10)

    # create the action server
    self._as = actionlib.SimpleActionServer('{}/follow_joint_trajectory'.format(name_space), FollowJointTrajectoryAction, self.goal_callback, False)
    self._as.start()

  def joint_states_callback(self, joint_states):
    try:
      self.tf_listener.waitForTransform("odom", "base_footprint", rospy.Time(), rospy.Duration(10))
    except TransformException as e:
      rospy.logwarn("base_contronller couldn't find odom frame")
      return
      
    t = self.tf_listener.getLatestCommonTime("odom", "base_footprint")
    position, quaternion = self.tf_listener.lookupTransform("odom", "base_footprint", t)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self._state.actual.positions = [position[0], position[1], euler[2]]

    try:
      self._state.actual.velocities = [joint_states.velocity[self.odom_x_joint_index], joint_states.velocity[self.odom_y_joint_index], joint_states.velocity[self.odom_z_joint_index]]
      # rospy.loginfo("base_controller found odom joint velocities")
    except IndexError as e:
      rospy.logwarn("base_controller couldn't find enough odom joint velocities in joint states!")
      return
    
    self._state.header.stamp = rospy.Time.now()
    self._state.header.frame_id = joint_states.header.frame_id
    self._state.joint_names = joint_states.name
    self.state_pub.publish(self._state)

  def goal_callback(self, goal):
    # helper variables
    success = True
    
    try:
      goal_odom_x_joint_index = goal.trajectory.joint_names.index(odom_x_joint)
      goal_odom_y_joint_index = goal.trajectory.joint_names.index(odom_y_joint)
      goal_odom_z_joint_index = goal.trajectory.joint_names.index(odom_z_joint)
    except:
      rospy.loginfo("base_controller aborted current goal")
      self._as.set_aborted()
      return

    # initialize cmd_vel and time index t
    cmd_vel = Twist()
    t = 0
    time_start = rospy.Time.now()
    while True:
      if self._as.is_preempt_requested():
        rospy.loginfo("The goal has been preempted")
        # the following line sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False
        break

      # goal error
      error = [[goal.trajectory.points[-1].positions[i] - self._state.actual.positions[i] for i in range(3)], [goal.trajectory.points[-1].velocities[i] - self._state.actual.velocities[i] for i in range(3)]]
      print("At " + str(t) + ": error = " + str(error))
      if sum([abs(error[i][j]) for i in range(2) for j in range(3)]) < 0.01:
        success = True
        break

      time_from_start = rospy.Time.now() - time_start
      if time_from_start.secs > 60:
        success = True
        break

      error_odom_x_pos = goal.trajectory.points[t].positions[goal_odom_x_joint_index] - self._state.actual.positions[0]
      error_odom_y_pos = goal.trajectory.points[t].positions[goal_odom_y_joint_index] - self._state.actual.positions[1]
      error_odom_z_pos = goal.trajectory.points[t].positions[goal_odom_z_joint_index] - self._state.actual.positions[2]

      error_odom_x_vel = goal.trajectory.points[t].velocities[goal_odom_x_joint_index] - self._state.actual.velocities[0]
      error_odom_y_vel = goal.trajectory.points[t].velocities[goal_odom_y_joint_index] - self._state.actual.velocities[1]
      error_odom_z_vel = goal.trajectory.points[t].velocities[goal_odom_z_joint_index] - self._state.actual.velocities[2]

      self._feedback.feedback.header.stamp = rospy.Time.now()
      self._feedback.feedback.header.frame_id = self._state.header.frame_id
      self._feedback.feedback.joint_names = self._state.joint_names
      self._feedback.feedback.desired.positions = goal.trajectory.points[t].positions
      self._feedback.feedback.desired.velocities = goal.trajectory.points[t].velocities
      self._feedback.feedback.desired.time_from_start = time_from_start
      self._feedback.feedback.actual.positions = self._state.actual.positions
      self._feedback.feedback.actual.velocities = self._state.actual.velocities
      self._feedback.feedback.actual.time_from_start = time_from_start
      self._feedback.feedback.error.positions = [error_odom_x_pos, error_odom_y_pos, error_odom_z_pos]
      self._feedback.feedback.error.velocities = [error_odom_x_vel, error_odom_y_vel, error_odom_z_vel]
      self._feedback.feedback.error.time_from_start = time_from_start
      
      # publish the feedback
      self._as.publish_feedback(self._feedback.feedback)
      
      # set goal velocites in map frame
      if t < len(goal.trajectory.points) and t >= 0:
        v_x = goal.trajectory.points[t].velocities[goal_odom_x_joint_index]
        v_y = goal.trajectory.points[t].velocities[goal_odom_y_joint_index]
        v_z = goal.trajectory.points[t].velocities[goal_odom_z_joint_index]
        t += 1

      if t == len(goal.trajectory.points):
        v_x = 0
        v_y = 0
        v_z = 0
        t = -1
        
      # add feedback control
      v_x += 0.5 * error_odom_x_pos + 0.1 * error_odom_x_vel
      v_y += 0.5 * error_odom_y_pos + 0.1 * error_odom_y_vel
      # v_z += 0.5 * error_odom_z_pos + 0.1 * error_odom_z_vel

      # transform velocities from map fram to base frame
      sin_z = math.sin(self._state.actual.positions[2])
      cos_z = math.cos(self._state.actual.positions[2])
      cmd_vel.linear.x = v_x * cos_z + v_y * sin_z
      cmd_vel.linear.y = -v_x * sin_z + v_y * cos_z
      cmd_vel.angular.z = v_z

      # publish the velocity
      self.cmd_vel_pub.publish(cmd_vel)
      rate.sleep()

    # set velocites to zero
    self.cmd_vel_pub.publish(Twist())

    if success:
      rospy.loginfo("The goal has been reached")
      self._result.result.error_string = "no error"
      self._as.set_succeeded(self._result.result)
    else:
      self._as.set_aborted(self._result.result)

if __name__ == '__main__':
  rospy.init_node("base_controller")
  name_space = rospy.get_param('~name_space')
  odom_x_joint = rospy.get_param('{}/odom_x_joint'.format(name_space))
  odom_y_joint = rospy.get_param('{}/odom_y_joint'.format(name_space))
  odom_z_joint = rospy.get_param('{}/odom_z_joint'.format(name_space))

  # publish info to the console for the user
  rospy.loginfo("base_controller starts")

  # start the base control
  BaseControl()

  # keep it running
  rospy.spin()
