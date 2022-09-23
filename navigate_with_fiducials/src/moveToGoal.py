#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
import message_filters
import math
import sys
import tf
from std_srvs.srv import Trigger

class odomPublisher():
    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.odomSub = rospy.Subscriber("/wheel_pose", PoseStamped, callback=self.handleOdom)
        rospy.wait_for_service('/core2/reset_odometry')
        self.odomResetter = rospy.ServiceProxy('/core2/reset_odometry', Trigger)
        self.odomResetter(Trigger())

    def handleOdom(self, msg):
        self.broadcaster.sendTransform(
            (msg.pose.position.x, msg.pose.position.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.z),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

class MoveToGoal:
    def __init__(self):
        self.odomSub = rospy.Subscriber("/wheel_pose", PoseStamped, callback=self.callback)
        self.anglePrecision = 0.08
        self.distancePrecision = 0.05
        self.twistPub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        self.goalSub = rospy.Subscriber("/goal", Pose, callback=self.setGoal)

        self.goalX = 0
        self.goalY = 0

        self.forwardRate = 0.3
        self.turnRate = 0.5

        self.stop = False

    def setGoal(self, msg):
        self.goalX = msg.position.x 
        self.goalY = msg.position.y 

    def callback(self, odom):
        print("callback")
        odomX = odom.pose.position.x 
        odomY = odom.pose.position.y 

        angleRequired = math.atan2((self.goalY - odomY), (self.goalX - odomX))
        
        quaternion = (odom.pose.orientation.x, odom.pose.orientation.y, odom.pose.orientation.z, odom.pose.orientation.w)
        angleCurrent = tf.transformations.euler_from_quaternion(quaternion)[2]

        print("Current angle: ", angleCurrent)
        print("Angle required: ", angleRequired)

        yawResponse = PID(angleCurrent, angleRequired, threshold=self.anglePrecision)
        yawResponse = yawResponse%2*math.pi
        # if self.goalX - odomX < 0:
        #     yawResponse += math.pi
        if yawResponse > math.pi:
            yawResponse -= 2*math.pi

        print("Yaw response: ", yawResponse)

        distance = math.sqrt(((self.goalX - odomX)**2 + (self.goalY - odomY)**2))
        print("Distance: ", distance)
        print("-"*16)
        if(yawResponse != 0):
            output = Twist()
            output.angular.z = yawResponse
            self.twistPub.publish(output)
        else:
            if distance > self.distancePrecision:
                output = Twist()
                output.linear.x = min(distance, self.forwardRate)
                self.twistPub.publish(output)
        

def PID(actual, goal, threshold=0):
    pGain = 1.0
    # iGain = 1
    # dGain = 1

    error = goal-actual
    response = pGain * error

    if abs(response) < threshold:
        return 0
    else:
        return response
    

def main():
    rospy.init_node("MoveToGoal")
    mover = MoveToGoal()
    # odomPublisher = OdomPublisher()

    rospy.spin()

if __name__=="__main__":
    main()