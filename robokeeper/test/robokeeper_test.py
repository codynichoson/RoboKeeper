#!/usr/bin/env python3
# PKG = 'robokeeper'

import unittest
import rospy
import numpy as np
from robokeeper.srv import Keeper
from robokeeper_library.robokeeper_functions import joint_lookup_table, traj_linear_reg

class RobokeeperTest(unittest.TestCase):

    def __init__(self, *args):
        super(RobokeeperTest,self).__init__(*args)
        rospy.init_node("robokeeper_test")
        rospy.sleep(3)

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

    def test_linear_reg(self): 
        # experimental data
        test_data_x = [
            [0.18055818, 0.15823179, 0.13166919, 0.07714387, 0.05425928, 0.00130307, -0.02248313, -0.05107477, -0.10842705, -0.09975091],
            [0.17978307, 0.15550394, 0.12235177, 0.05721768, 0.03153391, 0.03387083, -0.00786665, -0.04482902, -0.10263595, -0.13584961],
            [0.19367411, 0.17246582, 0.11381008, 0.08705496, 0.06641231, 0.00345005, -0.0206043, -0.05309508, 0.14674575, 0.17214538]
        ]
        test_data_y = [
            [-0.50462673, -0.48917394, -0.47541637, -0.44053934, -0.4238955, -0.39002688, -0.37935864, -0.36052673, -0.32708094, -0.32755835],
            [-0.57519872, -0.55924923, -0.54609925, -0.52790549, -0.51948879, -0.51969813, -0.50723612, -0.48986282, -0.47127546, -0.45903185],
            [-0.61137612, -0.59443004, -0.5648599, -0.54932695, -0.5325663, -0.50486167, -0.48727276, -0.46584977, -0.63188206, -0.57124508]
        ]
        y_predictions = [0.2734, -0.1302, 0.1235]  # y-coord predictions from experiments
        x_goal = -1.07  # x-coord of goal line for the above experiements

        for i, pred_known in enumerate(y_predictions):
            pred_calc = traj_linear_reg(test_data_x[i], test_data_y[i], x_goal) 
            self.assertEquals(pred_known, np.round(pred_calc, 4))

    def test_home(self):
        # wait for keep service to become available
        rospy.wait_for_service("keep")
        # call keep service
        kpr = rospy.ServiceProxy("keep", Keeper)
        resp = kpr(0.0)
        self.assertEquals(1, resp.code.val)

    def test_keep(self):
        # wait for keep service to become available
        rospy.wait_for_service("keep")
        # call keep service
        kpr = rospy.ServiceProxy("keep", Keeper)
        resp = kpr(0.2)
        self.assertEquals(1, resp.code.val)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('robokeeper', 'robokeeper_test', RobokeeperTest)
