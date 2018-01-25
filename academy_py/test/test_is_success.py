from academy_py.service import SelectJointService
from academy_msgs.srv import SelectJointResponse

import unittest
class IsSuccessTests(unittest.TestCase):
    def test_true(self):
        r = SelectJointResponse()
        r.success = True
        self.assertTrue(SelectJointService.is_success(r))

    def test_false(self):
        r = SelectJointResponse()
        r.success = False
        self.assertFalse(SelectJointService.is_success(r))

    def test_noobj(self):
        self.assertFalse(SelectJointService.is_success(None))
