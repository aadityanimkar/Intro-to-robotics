import rospy
from open_manipulator_msgs.msg import JointPosition, SetJointPosition

class OpenManipulatorController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('openmanipulator_controller', anonymous=True)
        
        # Publisher to send joint commands
        self.joint_pub = rospy.Publisher('/open_manipulator/goal_joint_space_path', SetJointPosition, queue_size=10)
        
        # Set the loop rate (10 Hz)
        self.rate = rospy.Rate(10)

    def move_to_joint_position(self, joint_names, joint_angles, time_from_start):
        # Create a SetJointPosition message
        joint_goal = SetJointPosition()
        joint_goal.joint_position.joint_name = joint_names
        joint_goal.joint_position.position = joint_angles
        joint_goal.path_time = time_from_start
        
        # Publish the joint goal
        self.joint_pub.publish(joint_goal)
        rospy.sleep(time_from_start + 0.5)  # Sleep to ensure the motion is completed

if __name__ == '__main__':
    try:
        # Initialize the controller
        controller = OpenManipulatorController()
        
        # Define the joint names (these are typical for OpenManipulatorX)
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Step 1: Move the robot to the default initial position
        default_joint_angles = [0.0, -1.05, 0.35, 0.70]  # Default "home" position for OpenManipulatorX
        time_to_default_position = 2.0  # Time to reach the default initial position
        controller.move_to_joint_position(joint_names, default_joint_angles, time_to_default_position)
        
        # Step 2: Move the robot to the desired joint positions
        target_joint_angles = [0.0, -0.5, 0.5, 0.0]  # Example target positions
        time_to_target_position = 2.0  # Time to reach the target positions
        controller.move_to_joint_position(joint_names, target_joint_angles, time_to_target_position)

    except rospy.ROSInterruptException:
        pass
