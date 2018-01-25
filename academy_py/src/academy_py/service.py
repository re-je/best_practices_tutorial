import rospy
from academy_msgs.srv import SelectJoint

class SelectJointService(object):
    @staticmethod
    def is_success(response):
        try:
            return response.success is True
        except:
            return False

    def __init__(self, service='select_joint'):
        self.proxy = rospy.ServiceProxy(service, SelectJoint)

    def wait(self, timeout):
        if timeout == 0:
            return
        self.proxy.wait_for_service(timeout)

    def select_joint(self, joint_name, timeout=None):
        try:
            self.wait(timeout)
            return self.is_success(self.proxy(joint_name))
        except Exception, e:
          rospy.logerr("Could not select joint '%s: %s'" % (joint_name, e))
          return False
