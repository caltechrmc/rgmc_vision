#!/usr/bin/env python

import numpy as np
import rospy
from tf import TransformBroadcaster, TransformListener
from typing import Tuple

from apriltag_ros.msg import AprilTagDetectionArray
from visualization_msgs.msg import Marker

TagDetectionCallbackArgs = Tuple[TransformListener, TransformBroadcaster]

last_tag_0_detection = None
TAGID2LETTER = {34: "A", 44: "B", 36: "C", 32: "D", 42: "E", 40: "F"}


def create_object_marker() -> Marker:
    """
    Create a visualization marker for the object.
    """
    marker = Marker()
    marker.header.frame_id = "cube_center"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = "package://camera_test/assets/meshes/cube_small.stl"
    marker.action = Marker.ADD
    marker.scale.x = 1e-3
    marker.scale.y = 1e-3
    marker.scale.z = 1e-3
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    return marker


def tag_detection_callback(msg: AprilTagDetectionArray, args: TagDetectionCallbackArgs):
    global last_tag_0_detection
    tf_listener, br = args

    if not msg.detections:
        return

    detected_tag_0 = False
    for det in msg.detections:
        tag_id = det.id[0]
        timestamp = det.pose.header.stamp
        cam_frame = det.pose.header.frame_id
        pose = det.pose.pose.pose
        pos = pose.position
        ori = pose.orientation

        if tag_id == 0:
            detected_tag_0 = True
            last_tag_0_detection = det
            # Define tabletop at tag_0 origin
            br.sendTransform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                timestamp,
                "tabletop",
                "tag_0",
            )
        else:
            # Broadcast each non-zero tag relative to its camera frame
            br.sendTransform(
                (pos.x, pos.y, pos.z),
                (ori.x, ori.y, ori.z, ori.w),
                timestamp,
                f"tag_{TAGID2LETTER.get(tag_id, tag_id)}",
                cam_frame,
            )

    if not detected_tag_0 and last_tag_0_detection:
        # Fallback: use last known pose of tag_0 relative to its camera
        last_pose = last_tag_0_detection.pose.pose.pose
        pos = last_pose.position
        ori = last_pose.orientation
        cam_frame = last_tag_0_detection.pose.header.frame_id

        # Keep tabletop at tag_0 origin
        br.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            "tabletop",
            "tag_0",
        )
        # Broadcast tag_0 relative to camera frame
        br.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            rospy.Time.now(),
            "tag_0",
            cam_frame,
        )


def visualizer():
    rospy.init_node("visualizer_multi_cam")
    tf_listener = TransformListener()
    br = TransformBroadcaster()

    # Publish static object marker once
    marker_pub = rospy.Publisher("object_marker", Marker, queue_size=10)
    obj_marker = create_object_marker()
    obj_marker.header.stamp = rospy.Time.now()
    marker_pub.publish(obj_marker)

    # Subscribe to merged detections topic
    rospy.Subscriber(
        "/merged_tag_detections", AprilTagDetectionArray,
        tag_detection_callback, (tf_listener, br), queue_size=10
    )

    rospy.spin()


if __name__ == "__main__":
    visualizer()
