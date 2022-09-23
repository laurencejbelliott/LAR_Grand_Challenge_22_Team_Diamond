"""
LAR Summer Camp 2022
Maintainer: Katherine James kajames@lincoln.ac.uk
Class to compute basic phenotyping of crops in the walled garden. Filters using vegetation index ExG, blob detection and use of the depth camera.
"""
from decimal import MAX_EMAX
import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class PhenoProc:
    """Class to implement looking for green, then stopping"""

    def __init__(self):

        # Subscribe
        self.cam_sub = rospy.Subscriber(
            "/d435i_camera/color/image_raw", Image, self.cam_callback
        )
        self.depth_sub = rospy.Subscriber(
            "/d435i_camera/aligned_depth_to_color/image_raw", Image, self.depth_callback
        )

        self.cam_pub = rospy.Publisher(
            "/d435i_camera/color/image_raw/compressed_filtered", Image, queue_size=0
        )
        self.depth_pub = rospy.Publisher(
            "/d435i_camera/color/image_raw/depth_filtered", Image, queue_size=0
        )

        self.bridge = CvBridge()  # Bridge used for converting msgs to cv2 image class

        # Properties
        self.image = None
        self.depth = None
        self.depth_mask = None
        self.depth_intrinsics = {}
        self.imgcounter = 0

    def depth_callback(self, msg):
        # format reply
        reply = Image()
        reply.header = msg.header

        depth = self.bridge.imgmsg_to_cv2(
            msg
        )  # Convert msg to actual cv2 image class (480, 640)

        criteria = ((depth < 1500) & (depth > 800)).astype(np.uint8)  # depth in mm
        depth = depth * criteria

        mask = (depth > 0).astype(np.uint8)

        self.depth_mask = mask

        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth))

    def cam_callback(self, msg):
        # format reply
        reply = Image()
        reply.header = msg.header

        image = self.bridge.imgmsg_to_cv2(
            msg, "bgr8"
        )  # Convert msg to actual cv2 image class

        # Filter based on depth
        if self.depth_mask is not None:
            image = image * np.expand_dims(self.depth_mask, 2)

        # EXG = 2 * G - B - R
        blue = image[:, :, 0]
        green = image[:, :, 1]
        red = image[:, :, 2]

        exg = 2 * green - blue - red
        exg = np.expand_dims(exg, axis=2)

        # Filter based on ExG
        kernel = np.ones((5, 5), np.float32) / 25
        blurred = cv2.filter2D(exg, -1, kernel)

        (T, threshInv) = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY_INV)
        thresh = 255 - threshInv

        criteria = (threshInv > 0).astype(np.uint8)  # convert non-zeros to 1s
        out_image = image * np.expand_dims(criteria, 2)

        # Detect crop height (pseudo)

        # nonzero = np.argwhere(out_image[:,:,0] != 0)

        kernel = np.ones((5, 5), np.uint8)
        connected = cv2.dilate(blurred, kernel, iterations=1)

        height = -1
        contours, hierarchy = cv2.findContours(
            image=connected, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE
        )
        # cv2.drawContours(
        #     image=out_image,
        #     contours=contours,
        #     contourIdx=0,
        #     color=(0, 255, 0),
        #     thickness=1,
        #     lineType=cv2.LINE_AA,
        # )
        # c = contours[0]
        counter = 0
        for c in contours:
            if cv2.contourArea(c) > 10000:
                rect = cv2.boundingRect(c)
                x, y, w, h = rect
                cv2.rectangle(out_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                height +=h
                counter+=1
        height = height/counter

        # cv2.line(out_image, (0, min_y), (640, min_y), (0, 255, 0), thickness=2)
        # cv2.line(out_image, (0, max_y), (640, max_y), (255, 0, 0), thickness=2)

        # Print to image
        average_exg = np.mean(exg[exg != 0])
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(
            out_image,
            "ExG: {:.2f}".format(average_exg),
            (10, 50),
            font,
            1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            out_image,
            f"Height: {height} px",
            (10, 100),
            font,
            1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # kernel = np.ones((5, 5), np.uint8)
        # thresh = cv2.erode(thresh, kernel, iterations=1)

        # params = cv2.SimpleBlobDetector_Params()
        # params.filterByArea = True
        # params.minArea = 1000
        # params.filterByCircularity = False
        # params.filterByConvexity = False

        # detector = cv2.SimpleBlobDetector_create(params)

        # keypoints = detector.detect(thresh)

        # image = cv2.drawKeypoints(
        #     image,
        #     keypoints,
        #     np.array([]),
        #     (0, 0, 255),
        #     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        # )

        # image = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        #cv2.imwrite(f"images/{self.counter}.jpg", blurred)
        self.imgcounter += 1

        self.cam_pub.publish(
            self.bridge.cv2_to_imgmsg(out_image, "bgr8")
        )  # Convert image back to bgr8 image msg, then publish


if __name__ == "__main__":
    rospy.init_node("pheno")
    node = PhenoProc()

    print("Filter node running!")
    rospy.spin()
    cv2.destroyAllWindows()
