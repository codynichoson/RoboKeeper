#!/usr/bin/env python3
# PKG = 'robokeeper'

import sys
import unittest
import rospy

from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robokeeper_library.robokeeper_functions import joint_lookup_table, traj_linear_reg

class RobokeeperTest(unittest.TestCase):

    def __init__(self, *args):
        super(RobokeeperTest,self).__init__(*args)
        rospy.init_node("robokeeper_test")
        rospy.sleep(3)

    def test_reset(self):
        # wait for reset service to become available
        rospy.wait_for_service("reset")
        # call reset service
        rst = rospy.ServiceProxy("reset",Empty)
        empty_resp = rst()

        JointState_msg = JointTrajectory()
        JointState_msg.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        value = [-3.450028078456225, 1.1812012540734964, 2.8532913410087053, 3.1416885303149615, -0.6757391589862014, -1.2594366618108317]
        JointTrajectoryPoint_msg = JointTrajectoryPoint()
        JointTrajectoryPoint_msg.positions = value
        JointTrajectoryPoint_msg.time_from_start.nsecs = 10000000
        JointState_msg.points = [JointTrajectoryPoint_msg]

        # wait for message
        data = rospy.wait_for_message("arm_controller/command",JointTrajectory)
        self.assertEquals(data,JointState_msg)
        # self.assertEquals(1,2)

    def test_whatever(self):
        # you can start writing your code here! :)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('robokeeper', 'robokeeper_test', RobokeeperTest)