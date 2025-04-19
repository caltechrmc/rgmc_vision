import numpy as np
import rospkg
import rospy
import os

from typing import Tuple

from tf import TransformBroadcaster, TransformListener

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu

TagDetectionCallbackArgs = Tuple[TransformListener, TransformBroadcaster]

first_imu_measurement = True
initial_imu_orientation = None

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
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    return marker

def tag_detection_callback(msg: AprilTagDetectionArray, args: TagDetectionCallbackArgs):
    global last_tag_0_detection
    
    detections = msg.detections
    (tf, br) = args
    #br.sendTransform([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], rospy.Time.now(), "table_top", "world")
    
    if len(detections) == 0:
        return
    
    detected_tag_0 = False
    
    for detection in detections:
        tag = detection.id[0]
        timestamp = detection.pose.header.stamp
        pose = detection.pose.pose.pose
        position = pose.position
        orientation = pose.orientation
        
        if tag == 0:
            detected_tag_0 = True
            last_tag_0_detection = detection

            br.sendTransform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                timestamp,
                "tabletop",
                "tag_0",
            )
        else:
            # We know the transformation from usb_cam_rgb_camera_optical_frame to tag_#
            
            """
            # Get transform from tag to cube center
            tag_frame_id = "tag_%s" % TAGID2LETTER[tag]
            t = tf.getLatestCommonTime(tag_frame_id, "cube_center")
            p, R = tf.lookupTransform(tag_frame_id, "cube_center", t)
            T_tag_to_cube = tf.fromTranslationRotation(p, R)
            
            # Get transform from camera to tag
            T_cam_to_tag = tf.fromTranslationRotation([position.x, position.y, position.z], [orientation.x, orientation.y, orientation.z, orientation.w])
            
            # Get transform from table top to cam
            t = tf.getLatestCommonTime("world", "usb_cam_rgb_camera_optical_frame")
            p, R = tf.lookupTransform("world", "usb_cam_rgb_camera_optical_frame", t)
            T_world_to_cam = tf.fromTranslationRotation(p, R)
            
            print(np.linalg.inv(T_world_to_cam @ T_cam_to_tag @ T_tag_to_cube) @ np.array([0, 0, 0, 1]))
            """
            
            
            
            #"""
            br.sendTransform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                timestamp,
                "tag_%s" % TAGID2LETTER[tag],
                "tag_%d" % tag,
            )
            #"""
        
        """
        x = position.x
        y = position.y
        z = position.z
        
        object_marker.pose = pose
        
        #br.sendTransform([x, y, z], (0.0, 0.0, 0.0, 1.0), timestamp, "object_base", "world")
        
        object_marker.header.stamp = timestamp
        object_marker.action = Marker.MODIFY
        object_publisher.publish(object_marker)
        """
        
    if not detected_tag_0:
        pose = last_tag_0_detection.pose.pose.pose
        position = pose.position
        orientation = pose.orientation
        
        br.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            "tabletop",
            "tag_0",
        )
        
        br.sendTransform(
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            rospy.Time.now(),
            "tag_0",
            "usb_cam_rgb_camera_optical_frame",
        )
    
def visualizer():
    rospy.init_node("visualizer")
    
    tf = TransformListener()
    br = TransformBroadcaster(queue_size=0)
    
    
    
    #frame_rate = rospy.Rate(10.0)
    
    object_publisher = rospy.Publisher("object_marker", Marker, queue_size=10)
    
    
    object_marker = create_object_marker()
    object_marker.header.stamp = rospy.Time.now()
    object_publisher.publish(object_marker)
    
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tag_detection_callback, (tf, br), queue_size=10)
    
    rospy.spin()
    #while not rospy.is_shutdown():
    #    br.sendTransform((0.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "object", "world")
    #    frame_rate.sleep()
        
if __name__ == "__main__":
    visualizer()