#! /usr/bin/env python
import rospy

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult, FollowJointTrajectoryActionGoal

class BaseControl(object):
  # create messages that are used to publish feedback/result
  _feedback = FollowJointTrajectoryActionFeedback()
  _result = FollowJointTrajectoryActionResult()

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("base_control_server", FollowJointTrajectoryAction, self.goal_callback, False)
    self._as.start()

  def goal_callback(self, goal):

    # helper variables
    rate = rospy.Rate(1)
    success = True

    # publish info to the console for the user
    rospy.loginfo("base_control starts")

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
      rospy.loginfo("Goal reached")
      self._as.set_succeeded()

if __name__ == '__main__':
  rospy.init_node("base_control")
  BaseControl()
  rospy.spin()