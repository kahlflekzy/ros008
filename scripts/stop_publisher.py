#! /usr/bin/python3

import behavior_tree_navigation_v4.msg as msgs

import rospy


class Publisher:
    """
    * Gets some poses I had stored in a particular file,
    * Parses them into PoseTime(s),
    * Publishes PoseTimeArray messages based on some selections made by the user.
    """

    def __init__(self):
        self.pub = rospy.Publisher("/stop_action_server/stop", msgs.Stop, queue_size=1)
        rospy.loginfo("Creating stop publisher node.")

    def __call__(self, *args, **kwargs):
        """
        Main loop

        :param args:
        :param kwargs:
        :return:
        """
        rospy.init_node("stop_publisher")
        while not rospy.is_shutdown():
            self.prompt()
            try:
                message = self.get_selection()
                self.pub.publish(message)
            except IndexError:
                rospy.loginfo("Wrong Input.")

    @staticmethod
    def get_selection() -> msgs.Stop:
        """
        Get user's selection and then choose selected poses for publishing.

        :return:
        """
        stop = msgs.Stop()
        try:
            f = int(input())
            if 1 <= f <= 5:
                stop.stop = f
            else:
                stop.stop = stop.CONTINUE
        except ValueError:
            stop.stop = stop.CONTINUE
        return stop

    @staticmethod
    def prompt():
        message = "\nEnter \n" \
                  "1. Stop robot and go to next goal.\n" \
                  "2. Stop robot and end task.\n" \
                  "3. End tasks after this goal.\n" \
                  "4. To continue.\n" \
                  "5. To attempt to cancel last instruction.\n"
        rospy.loginfo(message)


if __name__ == '__main__':
    node = Publisher()
    node()
