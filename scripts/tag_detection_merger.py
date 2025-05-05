#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray

class TagDetectionMerger:
    def __init__(self):
        rospy.init_node("tag_detection_merger")

        # Subscribers for each camera's detections
        self.sub1 = rospy.Subscriber("/usb_cam_1/tag_detections", AprilTagDetectionArray, self.callback_cam1)
        self.sub2 = rospy.Subscriber("/usb_cam_2/tag_detections", AprilTagDetectionArray, self.callback_cam2)

        # Publisher for the merged detections
        self.pub = rospy.Publisher("/merged_tag_detections", AprilTagDetectionArray, queue_size=10)

        self.latest_cam1 = None
        self.latest_cam2 = None

        rospy.loginfo("Tag Detection Merger node started.")

    def callback_cam1(self, msg):
        self.latest_cam1 = msg
        self.publish_merged()

    def callback_cam2(self, msg):
        self.latest_cam2 = msg
        self.publish_merged()

    def publish_merged(self):
        # If neither camera has published yet, do nothing
        if self.latest_cam1 is None and self.latest_cam2 is None:
            return

        merged = AprilTagDetectionArray()

        # Choose the newest timestamp for the header
        if self.latest_cam1 and self.latest_cam2:
            merged.header = (self.latest_cam1.header
                             if self.latest_cam1.header.stamp > self.latest_cam2.header.stamp
                             else self.latest_cam2.header)
        elif self.latest_cam1:
            merged.header = self.latest_cam1.header
        else:
            merged.header = self.latest_cam2.header

        # Concatenate the two detection lists
        if self.latest_cam1:
            merged.detections.extend(self.latest_cam1.detections)
        if self.latest_cam2:
            merged.detections.extend(self.latest_cam2.detections)

        self.pub.publish(merged)

if __name__ == "__main__":
    try:
        TagDetectionMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass