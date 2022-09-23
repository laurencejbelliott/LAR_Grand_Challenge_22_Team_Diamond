#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
import math
import tf
import json
import numpy as np

class nodeTraverser():
    def __init__(self):
        self.nextGoalPub = rospy.Publisher("/goal", Pose, queue_size=0)
        self.odomSub = rospy.Subscriber("/wheel_pose", PoseStamped, callback=self.odomCallback)
        # self.goalList = json.loads("~/.ros/moveGoals/path.json")
        self.goalList = [[2, 0], [2, 1], [0, 0]]
        self.loop = True
        self.nextNavGoal = [0, 0, 0]
        self.pathIndex = -1
        self.currentPos = [0, 0, 0]
        self.startOffset = False
        self.accuracyThreshold = 0.05

    def odomCallback(self, msg):
        if self.loop:
            self.pathIndex = 0
        if self.pathIndex >= len(self.goalList):
            return

        [x, y, theta] = [msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.z]
        if not self.startOffset:
            self.startOffset = [x, y, theta]
        self.currentPos = [x, y, theta]
        if self.at(self.nextNavGoal):
            print("Reached goal: ", self.nextNavGoal)
            self.pathIndex += 1
            self.nextNavGoal = self.goalList[self.pathIndex]
            print("Next goal: ", self.nextNavGoal)
            output = Pose()
            output.position.x = self.nextNavGoal[0] + self.startOffset[0]
            output.position.y = self.nextNavGoal[1] + self.startOffset[1]
            self.nextGoalPub.publish(output)

    def arrayDifference(self, array1, array2):
        assert len(array1) == len(array2), "Tried to subtract arrays of different length"
        array3 = []
        for i in range(len(array1)):
            array3.append(float(array1[i]) - float(array2[i]))
        return array3

    def at(self, goal):
        currentPosOffset = self.arrayDifference(self.currentPos, self.startOffset)
        # print("Current position: ", currentPosOffset)
        # print("Distance from next goal: ", sum(np.abs(self.arrayDifference(currentPosOffset[:1], goal[:1]))))
        if sum(np.abs(self.arrayDifference(currentPosOffset[:1], goal[:1]))) < self.accuracyThreshold:
            return True
        else:
            return False

def main():
    rospy.init_node("moveRoute")
    traverser = nodeTraverser()
    rospy.spin()

if __name__=="__main__":
    main()