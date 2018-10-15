#!/usr/bin/env python
from geometry_msgs.msg import Pose2D, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import math
import rospy



def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def distance(p1, p2):
    """Returns the distance between two Points/Vector3s.
    """
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class TaskTrajectory(object):
    DISTANCE_THRESHOLD = 0.02    
    def __init__(self, marker_pub):
        self._trajectory = []
        self._marker_pub = marker_pub
        rospy.on_shutdown(self.shutdown)

    def callback(self, msg):
        point = Point(msg.x, msg.y, 0.25)

        if (len(self._trajectory) == 0 or
                distance(self._trajectory[-1], point) > TaskTrajectory.DISTANCE_THRESHOLD):
            self._trajectory.append(point)
            # self.viz_breadcrumb()
            self.viz_path()

    # def viz_breadcrumb(self):
    #     msg = Marker(
    #         type=Marker.SPHERE_LIST,
    #         ns='breadcrumb',
    #         id=len(self._trajectory),
    #         points=self._trajectory,
    #         scale=Vector3(0.01, 0.01, 0.01),
    #         header=Header(frame_id='gazebo_world'),
    #         color=ColorRGBA(1.0, 0.0, 0.0, 0.0))
    #     self._marker_pub.publish(msg)

    def viz_path(self):
        msg = Marker(
            type=Marker.LINE_STRIP,
            ns='path',
            id=len(self._trajectory),
            points=self._trajectory,
            scale=Vector3(0.1, 0.1, 0.1),
            header=Header(frame_id='gazebo_world'),
            color=ColorRGBA(1.0, 0.0, 0.0, 1.0))
        self._marker_pub.publish(msg)
    
    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop Trajectory Viz Node")
        rospy.sleep(1)

def main():
    rospy.init_node('traj_viz')
    wait_for_time()
    marker_publisher = rospy.Publisher(
        'trajectory_marker', Marker, queue_size=5)
    trajectory = TaskTrajectory(marker_publisher)
    rospy.Subscriber('/Read_joint_state/quickie_state', Pose2D, trajectory.callback)

    rospy.spin()

    rospy.loginfo('Running until shutdown (Ctrl-C).')
    while not rospy.is_shutdown():
       trajectory.shutdown()
       rospy.sleep(0.5)

    rospy.loginfo('Node finished')


if __name__ == '__main__':
    main()