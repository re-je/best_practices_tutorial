#!/usr/bin/env python

import unittest

from academy_py.service import SelectJointService

from visualization_msgs.msg import Marker
import rospy

class TestSelectJointService(unittest.TestCase):
    def _callback(self, msg):
        self.msg = msg

    def setUp(self):
        self.sub = rospy.Subscriber("selected_joint_marker", Marker, self._callback, queue_size=1)
        self.msg = None

    def tearDown(self):
        self.sub.unregister()
        self.msg = None

    def run(self):
        srv = SelectJointService()

        try:
            srv.wait(1.0)
        except Exception:
            self.fail("Service not ready")

        rospy.sleep(0.5)
        msg = self.msg
        self.assertIsNone(msg)

        self.assertTrue(srv.select_joint("joint_1"))
        rospy.sleep(0.5)
        msg = self.msg
        self.assertEqual("link_1", msg.header.frame_id)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_selectjointservice')
    rostest.rosrun('academy_py', 'test_selectjointservice', TestSelectJointService)
