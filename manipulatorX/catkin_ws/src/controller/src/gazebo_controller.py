#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def move_to_position(joint_positions):
    """Publish joint positions to control the robot in Gazebo."""
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    
    for joint_name, position in zip(joint_names, joint_positions):
        pub = rospy.Publisher(f'/open_manipulator/{joint_name}_position_controller/command', Float64, queue_size=10)
        rospy.sleep(0.1)  # Allow publisher to initialize
        pub.publish(Float64(position))

def publish_joint_and_gripper_states(joint_positions, gripper_position):
    """Publish joint states and gripper state."""
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    gripper_command_pub = rospy.Publisher('/gripper/command', Float64, queue_size=10)

    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_state_msg.position = joint_positions
    joint_state_msg.velocity = []  # You can add velocity data if needed
    joint_state_msg.effort = []    # You can add effort data if needed

    gripper_command_msg = Float64()
    gripper_command_msg.data = gripper_position

    joint_state_pub.publish(joint_state_msg)
    gripper_command_pub.publish(gripper_command_msg)

def move_and_publish():
    rospy.init_node('move_and_publish_node', anonymous=True)

    # Define home and target positions
    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    target_position = [1.0, 0.5, -0.5, 1.0, 0.0, 0.0]
    gripper_position = 0.0  # Example gripper position

    rate = rospy.Rate(10)  # 10 Hz

    # Move to home position
    rospy.loginfo("Moving to home position...")
    move_to_position(home_position)
    
    # Wait for the robot to reach the home position
    rospy.sleep(2)  # Adjust sleep time as needed

    rospy.loginfo("Publishing home position...")
    publish_joint_and_gripper_states(home_position, gripper_position)
    
    # Continuously publish home position
    while not rospy.is_shutdown():
        publish_joint_and_gripper_states(home_position, gripper_position)
        rate.sleep()
        break  # Remove break to keep publishing until stopped

    # Move to target position
    rospy.loginfo("Moving to target position...")
    move_to_position(target_position)

    # Wait for the robot to reach the target position
    rospy.sleep(2)  # Adjust sleep time as needed

    rospy.loginfo("Publishing target position...")
    publish_joint_and_gripper_states(target_position, gripper_position)
    
    # Continuously publish target position
    while not rospy.is_shutdown():
        publish_joint_and_gripper_states(target_position, gripper_position)
        rate.sleep()
        break  # Remove break to keep publishing until stopped

    rospy.loginfo("Movement complete.")
    rospy.spin()

if __name__ == '__main__':
    try:
        move_and_publish()
    except rospy.ROSInterruptException:
        pass