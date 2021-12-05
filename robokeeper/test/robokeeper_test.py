#!/usr/bin/env python3
# PKG = 'robokeeper'

import unittest
import rospy

from robokeeper.srv import Keeper
from robokeeper_library.robokeeper_functions import joint_lookup_table, traj_linear_reg

class RobokeeperTest(unittest.TestCase):

    def __init__(self, *args):
        super(RobokeeperTest,self).__init__(*args)
        rospy.init_node("robokeeper_test")
        rospy.sleep(3)

    def test_home(self):
        # wait for keep service to become available
        rospy.wait_for_service("keep")
        # call keep service
        kpr = rospy.ServiceProxy("keep",Keeper)
        resp = kpr(0.0)
        self.assertEquals(1, resp.code.val)

    def test_keep(self):
        # wait for keep service to become available
        rospy.wait_for_service("keep")
        # call keep service
        kpr = rospy.ServiceProxy("keep",Keeper)
        resp = kpr(0.2)
        self.assertEquals(1, resp.code.val)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('robokeeper', 'robokeeper_test', RobokeeperTest)