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

    # def test_reset(self):
    #     # wait for reset service to become available
    #     rospy.wait_for_service("reset")
    #     # call reset service
    #     rst = rospy.ServiceProxy("reset",Empty)
    #     empty_resp = rst()

    #     JointState_msg = JointTrajectory()
    #     JointState_msg.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
    #     value = [-3.450028078456225, 1.1812012540734964, 2.8532913410087053, 3.1416885303149615, -0.6757391589862014, -1.2594366618108317]
    #     JointTrajectoryPoint_msg = JointTrajectoryPoint()
    #     JointTrajectoryPoint_msg.positions = value
    #     JointTrajectoryPoint_msg.time_from_start.nsecs = 10000000
    #     JointState_msg.points = [JointTrajectoryPoint_msg]

    #     # wait for message
    #     data = rospy.wait_for_message("arm_controller/command",JointTrajectory)
    #     self.assertEquals(data,JointState_msg)
    #     # self.assertEquals(1,2)

    def test_lookup_table(self):
        y_tests = [0.09, -0.03, 0.18, -0.23]  # test these y-coords
        expected_angles = {
            0.09: [-2.980615632032212, 1.0738193218849965, 3.1179111024732222, -0.22473504379450288, 0.5154332745047984, 1.6421765486826985, 0.10124582177772824],
            -0.03: [-3.183874289389015, 1.0784214046930751, 3.219923938052297, 0.09050762855887828, 0.4126534251243773, 1.5179203128645773, 0.10124582177772824],
            0.18: [-2.8425531477898556, 1.090693625514618, 3.002092018469912, -0.3850409282759059, 0.5998047926529053, 1.6651869627230913, 0.10124582177772824],
            -0.23: [-3.5788863970824245, 1.141316536403482, 3.258274628119618, 0.9112123959995543, 0.3367190587910811, 1.0446727974338323, 0.10124582177772824]
        }
        for test in y_tests: 
            self.assertEquals(joint_lookup_table(test), expected_angles[test])

if __name__ == '__main__':
    import rostest
    rostest.rosrun('robokeeper', 'robokeeper_test', RobokeeperTest)