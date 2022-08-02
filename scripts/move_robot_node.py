#! /usr/bin/python3
import base64

import behavior_tree_navigation_v4.msg as msgs
import behavior_tree_navigation_v3.msg as msgs_v3
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID

import actionlib
import rospy


class ActionServer:
    """
    * Move A Robot
    * Halt the robot if preempted
    * Wait for some time as specified by the goal if robot get to goal and was not preempted.
    """

    def __init__(self, server_name="move_robot"):
        """"""
        self.task = None

        self.shutdown = False
        self.goal = GoalStatus()

        self.cancel_goal_publisher = None
        self.goal_publisher = None

        self.init_subscribers()
        self.init_publishers()

        self.execute_goal = actionlib.SimpleActionServer(server_name, msgs.MoveRobotAction, self.execute, False)

        self.execute_goal.start()

    def execute(self, action: msgs.MoveRobotGoal):
        """
        deserializes action goal, gets pose time stamped, extracts pose stamped, publishes to move the robot,
        loops while waiting for indication that robot got to goal.

        If there is a preempt signal from the client, task is stopped by sending the goal to a cancel topic.

        Waits for some time as specified in the pose_time, if robot successfully got to destination (no preemption).

        :param action:
        :return:
        """
        self.deserialize(action.goal)

        feedback = msgs.MoveRobotFeedback()
        result = msgs.MoveRobotResult()

        self.goal = GoalStatus()  # might have been forced set by other callers.

        pose_stamped = self.get_pose_stamped()

        self.goal_publisher.publish(pose_stamped)
        preempt = False
        while self.goal.status != GoalStatus.SUCCEEDED and not self.shutdown:
            if self.execute_goal.is_preempt_requested():
                # public a signal to cancel goal.
                self.cancel_goal_publisher.publish(self.goal.goal_id)
                preempt = True
                break

            feedback.status = self.goal.status
            self.execute_goal.publish_feedback(feedback)
        self.goal.goal_id.id = ''
        self.goal.status = GoalStatus.PENDING
        if not preempt:
            rospy.loginfo(f"Got to Pose. Waiting for {self.task.pose_time.duration} seconds.")
            rospy.sleep(self.task.pose_time.duration)
        result.result = 0
        self.execute_goal.set_succeeded(result)

    def init_subscribers(self):
        """Define topics for subscription"""
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.worker)
        rospy.loginfo("Initialised MoveRobot subscribers.")

    def init_publishers(self):
        """
        Define publishers.
        One moves the robot, the other cancels the motion command.

        :return:
        """
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.cancel_goal_publisher = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        rospy.sleep(2)

    def __call__(self, *args, **kwargs):
        rospy.on_shutdown(self.handle_shutdown)
        rospy.spin()

    def handle_shutdown(self):
        """"""
        rospy.loginfo("Cleaning Up.")
        self.shutdown = True

    def worker(self, goal_array_status):
        """"""
        if len(goal_array_status.status_list) != 0:
            goal = goal_array_status.status_list[-1]
            if self.goal.goal_id.id == '' and goal.status != GoalStatus.SUCCEEDED:
                self.goal.goal_id.id = goal.goal_id.id
                self.goal.status = goal.status
            elif self.goal.goal_id.id == goal.goal_id.id:
                self.goal.status = goal.status
                self.goal.text = goal.text

    def deserialize(self, data):
        """
        Deserialize data: str into a PoseTime.
        First converts string into a byte, then uses base64 to decode
        the byte into it's original format.

        :param data:
        :return:
        """
        # rospy.loginfo(f"Deserializing data in MoveRobot: \n{data}")
        self.task = msgs.PoseTimeStamped()
        enc = 'utf-8'
        bytes_ = bytes(data, encoding=enc)
        base64_bytes = base64.b64decode(bytes_)
        self.task.deserialize(base64_bytes)

    def get_pose_stamped(self):
        """
        Creates a PoseStamped msg from a PoseTimeStamped msg.

        :return:
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.task.header.frame_id
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = self.task.pose_time.pose

        return pose_stamped


if __name__ == '__main__':
    rospy.init_node("move_robot_action_server")
    rospy.loginfo("Initialised MoveRobot ActionServer Node.")

    server = ActionServer()
    server()
