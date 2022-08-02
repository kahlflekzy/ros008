#! /usr/bin/python3
import behavior_tree_navigation_v4.msg as msgs

import actionlib
import rospy


class ActionsServer:
    """
    Creates a server for getting stop codes via a subscriber.
    Stop robot if needs be.
    """

    def __init__(self):
        """"""
        self.stop_codes = []  # todo maybe we shouldn't keep a list
        self.execute_goal = actionlib.SimpleActionServer("stop_server", msgs.StopAction, self.execute, False)
        self.execute_goal.start()
        self.init_subscribers()

    def execute(self, action: msgs.StopGoal):
        """
        Gets task: PoseTimeArray or index of goal: PoseTime in task, and returns a goal: PoseTime.
        Returns Stop::CONTINUE if index is invalid or no stop message has been received.

        :param action:
        :return:
        """
        result = msgs.StopResult()
        if len(self.stop_codes) == 0:
            result.result = msgs.Stop.CONTINUE
        else:
            stop = self.stop_codes.pop()
            result.result = stop.stop
        self.execute_goal.set_succeeded(result)

    def __call__(self, *args, **kwargs):
        rospy.spin()

    def init_subscribers(self):
        """Define topics for subscription"""
        rospy.Subscriber("/stop_action_server/stop", msgs.Stop, lambda data: self.stop_codes.append(data))
        rospy.loginfo("Initialised StopCode subscriber.")


if __name__ == '__main__':
    rospy.init_node("stop_action_server")
    rospy.loginfo("Initialised StopActionServer Node.")

    server = ActionsServer()
    server()
