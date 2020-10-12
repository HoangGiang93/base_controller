#! /usr/bin/env python
import rospy

import actionlib

from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult, FollowJointTrajectoryActionGoal
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped
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
    self.tf_broadcaster = tf.TransformBroadcaster()

    # create state publisher
    self.state_pub = rospy.Publisher('{}/state'.format(name_space), JointTrajectoryControllerState, queue_size=10)
    self.state_sub = rospy.Subscriber('base/joint_states', JointState, self.joint_states_callback, queue_size=10)

    # error_pos
    self.error_traj = [0,0,0]

    # create the action server
    self._as = actionlib.SimpleActionServer('{}/follow_joint_trajectory'.format(name_space), FollowJointTrajectoryAction, self.goal_callback, False)
    self._as.start()

  def get_odom_origin(self):
    try:
      self.tf_listener.waitForTransform("map", "odom", rospy.Time(), rospy.Duration(10))
    except TransformException as e:
      rospy.logwarn("base_controller couldn't find odom frame")

    t = self.tf_listener.getLatestCommonTime("map", "odom")
    return self.tf_listener.lookupTransform("map", "odom", t)

  def joint_states_callback(self, joint_states):
    # joint_states is based on odom, however when a goal is received, the robot will move, odom will be moved so joint_states must be based on odom_origin
    try:
      self.tf_listener.waitForTransform("odom_origin", "base_footprint", rospy.Time(), rospy.Duration(1))
      self.tf_listener.waitForTransform("base_footprint", "base_footprint_goal", rospy.Time(), rospy.Duration(1))

    except TransformException as e:
      # rospy.logwarn("base_controller couldn't find odom_origin frame, use odom frame instead") # for debug
      # base_controller couldn't find odom_origin frame, so odom frame will be used for joint_states
      t = self.tf_listener.getLatestCommonTime("odom", "base_footprint")
      position, quaternion = self.tf_listener.lookupTransform("odom", "base_footprint", t)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      self._state.actual.positions = [position[0], position[1], euler[2]]

    else:
      # joint_states makes the transformation from odom_origin to base_footprint
      t = self.tf_listener.getLatestCommonTime("odom_origin", "base_footprint")
      position, quaternion = self.tf_listener.lookupTransform("odom_origin", "base_footprint", t)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      self._state.actual.positions = [position[0], position[1], euler[2]]

      # error_traj is the transformation from base_footprint to base_footprint_goal
      t = self.tf_listener.getLatestCommonTime("base_footprint", "base_footprint_goal")
      position, quaternion = self.tf_listener.lookupTransform("base_footprint", "base_footprint_goal", t)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      self.error_traj[0] = position[0]
      self.error_traj[1] = position[1]
      self.error_traj[2] = euler[2]

    try:
      self._state.actual.velocities = [joint_states.velocity[self.odom_x_joint_index], joint_states.velocity[self.odom_y_joint_index], joint_states.velocity[self.odom_z_joint_index]]
    except IndexError as e:
      rospy.logwarn("base_controller couldn't find enough odom joint velocities in joint states!")
      self._as.set_aborted()
      return
    
    # publish joint_states
    self._state.header.stamp = rospy.Time.now()
    self._state.header.frame_id = joint_states.header.frame_id
    self._state.joint_names = joint_states.name
    self.state_pub.publish(self._state)

  def goal_callback(self, goal):
    # helper variables
    success = True
    rate = rospy.Rate(50)
    
    try:
      goal_odom_x_joint_index = goal.trajectory.joint_names.index(odom_x_joint)
      goal_odom_y_joint_index = goal.trajectory.joint_names.index(odom_y_joint)
      goal_odom_z_joint_index = goal.trajectory.joint_names.index(odom_z_joint)
    except:
      rospy.loginfo("base_controller aborted current goal")
      self._as.set_aborted()
      return

    odom_origin_pos, odom_origin_quat = self.get_odom_origin()

    # initialization
    cmd_vel = Twist()
    t = 0
    t_finish = 0
    time_start = rospy.Time.now()

    while True:
      if self._as.is_preempt_requested():
        rospy.loginfo("The goal has been preempted")
        # the following line sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False
        break

      time_from_start = rospy.Time.now() - time_start

      # set goal velocites in map frame
      if t < len(goal.trajectory.points) and t >= 0:
        for point_index in range(t,len(goal.trajectory.points)):
          if goal.trajectory.points[point_index].time_from_start < time_from_start:
            t += 1
          else:
            break

      if t == len(goal.trajectory.points):
        t = -1
        time_finish = rospy.Time.now()

      if t == -1:
        time_from_finish = rospy.Time.now() - time_finish
        if time_from_finish.secs > 10:
          success = True
          break

      v_x = goal.trajectory.points[t].velocities[goal_odom_x_joint_index]
      v_y = goal.trajectory.points[t].velocities[goal_odom_y_joint_index]
      v_z = goal.trajectory.points[t].velocities[goal_odom_z_joint_index]
      
      self.tf_broadcaster.sendTransform(odom_origin_pos,
        odom_origin_quat,
        rospy.Time.now(),
        "odom_origin",
        "map")

      goal_pos = [goal.trajectory.points[t].positions[goal_odom_x_joint_index], goal.trajectory.points[t].positions[goal_odom_y_joint_index], 0]
      goal_quat = tf.transformations.quaternion_from_euler(0, 0, goal.trajectory.points[t].positions[goal_odom_z_joint_index])
      self.tf_broadcaster.sendTransform(goal_pos,
        goal_quat,
        rospy.Time.now(),
        "base_footprint_goal",
        "odom_origin")

      # goal error
      if t == -1:
        if abs(self.error_traj[0]) + abs(self.error_traj[1]) + abs(self.error_traj[2]) < 0.001:
          success = True
          break
      print("At " + str(t) + ": error_traj = " + str(self.error_traj))

      error_odom_x_pos = self.error_traj[0]
      error_odom_y_pos = self.error_traj[1]
      error_odom_z_pos = self.error_traj[2]

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

      # transform velocities from map frame to base frame and add feedback control
      sin_z = math.sin(self._state.actual.positions[2])
      cos_z = math.cos(self._state.actual.positions[2])
      cmd_vel.linear.x = v_x * cos_z + v_y * sin_z + 2 * error_odom_x_pos + 0.5 * error_odom_x_vel
      cmd_vel.linear.y = -v_x * sin_z + v_y * cos_z + 2 * error_odom_y_pos + 0.5 * error_odom_y_vel
      cmd_vel.angular.z = v_z + 2 * error_odom_z_pos + 0.5 * error_odom_z_vel

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
