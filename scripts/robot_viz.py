#!/usr/bin/env python
from geometry_msgs.msg import Pose2D, Pose
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import math
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class ObjectViz(object):
    
    def __init__(self, marker_pub, robot_pub):
        self._object_pose    = []
        self._marker_pub     = marker_pub    
        self._robot_pub      = robot_pub
        rospy.on_shutdown(self.shutdown)


    def callback(self, msg):
        x = msg.x
        y = msg.y
        theta = msg.theta
        self._robot_pose = [x,y,theta];
        self.viz_cube()
        self.pub_pose()

    def viz_cube(self):
       marker = Marker()
       marker.header.frame_id = "/gazebo_world"
       marker.type = marker.CUBE
       marker.action = marker.ADD
       marker.scale.x = 0.78
       marker.scale.y = 0.66
       marker.scale.z = 1.0
       marker.color.a = 1.0
       marker.color.r = 0.5
       marker.color.g = 0.5
       marker.color.b = 0.5
       marker.pose.orientation.w = 1.0
       marker.pose.position.x = self._robot_pose[0]
       marker.pose.position.y = self._robot_pose[1]
       marker.pose.position.z = 0.5
       self._marker_pub.publish(marker)
    
    def pub_pose(self):
      robot = Pose()
      robot.position.x = self._robot_pose[0]
      robot.position.y = self._robot_pose[1]
      robot.position.z = 0.0
      robot.orientation.x = 0
      robot.orientation.y = 0
      robot.orientation.z = 0
      robot.orientation.w = 1
      self._robot_pub.publish(robot)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop Robot Viz Node")
        rospy.sleep(1)

def main():
    rospy.init_node('robot_viz')
    wait_for_time()
    marker_publisher = rospy.Publisher('object_marker', Marker, queue_size=5)
    robot_publisher  = rospy.Publisher('wheelchair_pose', Pose, queue_size=5)
    
    objectViz = ObjectViz(marker_publisher, robot_publisher)
    rospy.Subscriber('/Read_joint_state/quickie_state', Pose2D, objectViz.callback)
    rospy.spin()

    rospy.loginfo('Running until shutdown (Ctrl-C).')
    while not rospy.is_shutdown():
       objectViz.shutdown()
       rospy.sleep(0.5)

    rospy.loginfo('Node finished')


if __name__ == '__main__':
    main()