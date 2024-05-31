#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix
import numpy as np
from ros_igtl_bridge.msg import igtlpoint  # Import the correct message type
import moveit_commander
import moveit_msgs.msg

ENTRY_POINT_NAME ="MarkupsFiducial_3-1"
TARGET_POINT_NAME ="MarkupsFiducial_3-2"

entry_point=None
target_point=None

def compute_orientation(entry_point, target_point):
    # Compute the direction vector
    direction = np.array([target_point[0] - entry_point[0], 
                          target_point[1] - entry_point[1], 
                          target_point[2] - entry_point[2]])
    direction_norm = np.linalg.norm(direction)
    direction = direction / direction_norm  # Normalize the direction vector

    # Create a rotation matrix that aligns the z-axis with the direction vector
    z_axis = np.array([0, 0, 1])
    v = np.cross(z_axis, direction)
    c = np.dot(z_axis, direction)
    s = np.linalg.norm(v)

    # If the direction is exactly the z-axis or opposite
    if s == 0:
        if c > 0:
            rotation_matrix = np.identity(3)
        else:
            rotation_matrix = np.diag([1, 1, -1])
    else:
        vx = np.array([[0, -v[2], v[1]], 
                       [v[2], 0, -v[0]], 
                       [-v[1], v[0], 0]])
        rotation_matrix = np.identity(3) + vx + np.dot(vx, vx) * ((1 - c) / (s ** 2))

    # Convert rotation matrix to quaternion
    rotation_matrix_4x4 = np.identity(4)
    rotation_matrix_4x4[:3, :3] = rotation_matrix
    quaternion = quaternion_from_matrix(rotation_matrix_4x4)

    return quaternion

def ras_to_ros_position(ras_position):
    # Convert mm to m and apply the coordinate system transformation
    x_ros = -ras_position[0] * 0.001  # Convert mm to m
    y_ros = -ras_position[1] * 0.001  # Convert mm to m
    z_ros = ras_position[2] * 0.001   # Convert mm to m
    return [x_ros, y_ros, z_ros]

def point_callback(msg):
    global entry_point, target_point
    rospy.loginfo(f"Received point: name = {msg.name}")

    # Access the entry point and target point by their names
    if msg.name.strip() ==ENTRY_POINT_NAME:
            entry_point = msg.pointdata
    elif msg.name.strip() == TARGET_POINT_NAME:
            target_point = msg.pointdata

    if entry_point is None or target_point is None:
        rospy.logwarn("Could not find both entry and target points in the message")
        return

    # Print the received points in RAS coordinates (mm)
    rospy.loginfo("Received entry point (RAS, mm): x=%f, y=%f, z=%f", entry_point.x, entry_point.y, entry_point.z)
    rospy.loginfo("Received target point (RAS, mm): x=%f, y=%f, z=%f", target_point.x, target_point.y, target_point.z)

    # Convert points from RAS (mm) to ROS (m) coordinates
    entry_point_ros = ras_to_ros_position([entry_point.x, entry_point.y, entry_point.z])
    target_point_ros = ras_to_ros_position([target_point.x, target_point.y, target_point.z])

    # Print the converted points in ROS coordinates (m)
    rospy.loginfo("Converted entry point (ROS, m): x=%f, y=%f, z=%f", entry_point_ros[0], entry_point_ros[1], entry_point_ros[2])
    rospy.loginfo("Converted target point (ROS, m): x=%f, y=%f, z=%f", target_point_ros[0], target_point_ros[1], target_point_ros[2])

    # Compute the orientation
    quaternion = compute_orientation(entry_point_ros, target_point_ros)

    # Create a Pose message
    pose = Pose()
    pose.position.x = entry_point_ros[0]*100
    pose.position.y = entry_point_ros[1]*100
    pose.position.z = entry_point_ros[2]*100
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    # Publish the pose
    pose_pub.publish(pose)

    # Move the robot to the pose
    move_group.set_pose_target(pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == '__main__':
    rospy.init_node('move_robot_to_pose')

    # Initialize the moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the RobotCommander and MoveGroupCommander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "my_robot4"  # Replace with your planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Publisher for the needle pose
    pose_pub = rospy.Publisher('/needle_pose', Pose, queue_size=10)

    # Subscriber to the IGTL_POINT_IN topic
    rospy.Subscriber('/IGTL_POINT_IN', igtlpoint, point_callback)

    rospy.spin()
