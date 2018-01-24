#include <ros/ros.h>
#include <academy_msgs/SelectJoint.h>

#include <gtest/gtest.h>

TEST(TestSelectJoint, SelectJoint)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<academy_msgs::SelectJoint>("select_joint");

  ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

  academy_msgs::SelectJoint srv;

  srv.request.joint_name = "";
  ASSERT_TRUE(client.call(srv));
  ASSERT_FALSE(srv.response.success);


  srv.request.joint_name = "link_1";
  ASSERT_TRUE(client.call(srv));
  ASSERT_FALSE(srv.response.success);


  srv.request.joint_name = "joint_1";
  ASSERT_TRUE(client.call(srv));
  ASSERT_TRUE(srv.response.success);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_select_joint");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
