#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from std_srvs.srv import Empty
import math

class odomPublisher():
    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.odomSub = rospy.Subscriber("/wheel_pose", PoseStamped, callback=self.handleOdom)

        # rospy.wait_for_service("/fiducials/clear_map", Empty)

    def handleOdom(self, msg):
        self.broadcaster.sendTransform(
            (msg.pose.position.x, msg.pose.position.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.z),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

def main():
    rospy.init_node("OdomPublisher", anonymous=True)
    publisher = odomPublisher()
    print("Running")
    rospy.spin()

if __name__ == "__main__":
    main()

"""
1) Publish fixed map frame from vidusials
2) Make odometry relative to the fixed map
3) Update odometry to remove error when transform from vidusial to base_link is possible

a) publish odometry to [odom] frame
b) TF from [odom] to [map]
c) update b) whenever viduasial transform available
"""