#! /usr/bin/python3
import base64
from io import BytesIO

import behavior_tree_navigation_v4.msg as msgs
import behavior_tree_navigation_v3.msg as msgs_v3
from geometry_msgs.msg import PoseStamped

import actionlib
import rospy


class TasksActionsServer:
    """
    Creates a server for getting tasks [PoseTimeArray] and returning goals [PoseTime].
    """

    def __init__(self):
        """"""
        self.tasks = None

        self.execute_goal = actionlib.SimpleActionServer("tasks_server", msgs.TaskAction, self.execute, False)

        self.execute_goal.start()

    def execute(self, action: msgs.TaskGoal):
        """
        Gets task: PoseTimeArray or index of goal: PoseTime in task, and returns a goal: PoseTimeStamped.
        Returns "0" if index is invalid or task has not been received.
        Convert PoseTime to PoseTimeStamped

        :param action:
        :return:
        """
        tasks: str = action.task
        result = msgs.TaskResult()
        if tasks.isdigit():
            index = int(tasks)
            if self.tasks is None or index >= len(self.tasks.poses):
                result.goal = "0"
            else:
                pose_time = self.tasks.poses[index]
                pose_time_stamped = self.to_pose_time_stamped(pose_time)
                result.goal = self.serialize(pose_time_stamped)
                # delete task after last PoseTime has been sent. Maybe this is an overkill?
                if index + 1 == len(self.tasks.poses):
                    self.tasks = None
        else:
            self.deserialize(tasks)
            pose_time = self.tasks.poses[0]
            pose_time_stamped = self.to_pose_time_stamped(pose_time)
            result.goal = self.serialize(pose_time_stamped)
            if len(self.tasks.poses) == 1:
                self.tasks = None
        result.goal += ">1" if self.tasks else ">0"
        # rospy.loginfo(f"goal: \n{result.goal}")
        self.execute_goal.set_succeeded(result)

    def __call__(self, *args, **kwargs):
        rospy.spin()

    def deserialize(self, data):
        """
        Deserialize data: str into a task.
        First converts string into a byte, then uses base64 to decode
        the byte into it's original format.

        :param data:
        :return:
        """
        self.tasks = msgs_v3.PoseTimeArray()
        enc = 'utf-8'
        bytes_ = bytes(data, encoding=enc)
        base64_bytes = base64.b64decode(bytes_)
        self.tasks.deserialize(base64_bytes)

    @staticmethod
    def serialize(data: msgs.PoseTimeStamped) -> str:
        """
        Serialize data and then uses base64 to encode the serialized data into a printable format.

        :param data:
        :return: serialized_string
        """
        buff = BytesIO()
        data.serialize(buff)
        serialized_bytes = buff.getvalue()
        enc = 'utf-8'
        base64_bytes = base64.b64encode(serialized_bytes)
        serialized_string = str(base64_bytes, encoding=enc)
        return serialized_string

    def to_pose_time_stamped(self, pose_time):
        pose_time_stamped = msgs.PoseTimeStamped()
        pose_time_stamped.header = self.tasks.header
        pose_time_stamped.pose_time = pose_time
        return pose_time_stamped


if __name__ == '__main__':
    rospy.init_node("tasks_actions_server")
    rospy.loginfo("Initialised TasksActionsServer Node.")

    server = TasksActionsServer()
    server()
