#!/usr/bin/env python
import rospy
from nav_msgs.msg       import Odometry
from geometry_msgs.msg  import Pose
from gazebo_msgs.msg    import ModelStates

class gazebo_state_to_odom:

  def __init__(self):
    self.last_echo_time = rospy.get_rostime()
    self.params()
    self.pubs()
    self.subs()

  def params(self):
    self.model_name = rospy.get_param('~model_name', 'none')
    self.rate       = rospy.get_param('~rate', 1)
    frame_id        = rospy.get_param('~frame_id', 'map')
    child_frame_id  = rospy.get_param('~child_frame_id', self.model_name)
    self.odom = Odometry()
    self.odom.header.frame_id = frame_id
    self.odom.child_frame_id = child_frame_id
    print("model name: " + self.model_name)

  def subs(self):
    rospy.Subscriber("model_states", ModelStates, self.modelStatesCallback)

  def pubs(self):
    self.pub = rospy.Publisher('odometry/gazebo', Odometry, queue_size=10)

  def modelStatesCallback(self, model_states_msg):
    if rospy.get_rostime() - self.last_echo_time < rospy.Duration(1/self.rate):
      return
    # Find our model.
    found_match = False
    names = model_states_msg.name
    poses = model_states_msg.pose
    for name,pose in zip(names,poses):
      if name == self.model_name:
        found_match = True
        break
    if not found_match:
      print('Unable to find ' + self.model_name + ' in model states')
      return
    self.last_echo_time = rospy.get_rostime()
    # Publish.
    self.odom.header.seq = self.odom.header.seq + 1
    self.odom.header.stamp = rospy.get_rostime()
    self.odom.pose.pose = pose
    self.pub.publish(self.odom)

if __name__ == '__main__':
  print("Initialising node")
  rospy.init_node('gazebo_state_to_odom', anonymous=True)
  print("Creating object")
  obj = gazebo_state_to_odom()
  rospy.spin()
