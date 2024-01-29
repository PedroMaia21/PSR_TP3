#!/usr/bin/env python3

import rospy
import rospkg
import subprocess
import numpy as np
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped

class RespawnRobot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('respawn_robot', anonymous=True)

        # Wait for the services to be available
        rospy.wait_for_service('/gazebo/delete_model')
        rospy.wait_for_service('/gazebo/spawn_urdf_model')

        # Define service proxies
        self.delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        self.pose_publisher = rospy.Publisher('/your_robot/pose_estimate', PoseWithCovarianceStamped, queue_size=1)

    def find_urdf_path(self, package_name, urdf_file):
        # Find the path to the URDF file using rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)
        urdf_path = f"{package_path}/urdf/{urdf_file}"  # Adjust the path structure as needed
        return urdf_path

    def delete_and_spawn_robot(self):
        # Delete the robot model
        try:
            self.delete_model_service('turtlebot3')  # Replace 'turtlebot3' with the actual model name
            rospy.loginfo("Robot model deleted successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to delete robot model: %s", str(e))
            return

        # Wait for a short time to ensure the model is deleted before spawning again
        rospy.sleep(2)

         # Find the path to the URDF file using rospkg
        package_name = 'robutler1_description'  # Replace with the name of your robot's ROS package
        urdf_file = 'turtlebot3_manipulation_robot.urdf.xacro'  # Replace with the actual URDF file name

        urdf_path = self.find_urdf_path(package_name, urdf_file)

        # Spawn the robot model
        try:

            localization_node_name = 'amcl'
            kill_command = ['rosnode', 'kill', localization_node_name]
            subprocess.call(kill_command)

            roslaunch_command = ['roslaunch', 'robutler1_bringup', 'respawn.launch']
            process = subprocess.Popen(roslaunch_command)
            process.wait()

            roslaunch_command = ['roslaunch', 'robutler1_navigation', 'localization.launch']
            process = subprocess.Popen(roslaunch_command)
            process.wait()

            #  #After spawning, send a pose estimate
            # pose_estimate = PoseWithCovarianceStamped()
            # pose_estimate.header.stamp = rospy.Time.now()
            # pose_estimate.header.frame_id = 'map'
            # pose_estimate.pose.pose.position.x = 0#8.934732
            # pose_estimate.pose.pose.position.y = 0#-1.828018
            # pose_estimate.pose.pose.position.z = 0.0

            # roll = 0.0
            # pitch = 0.0
            # yaw = 3.14
            # qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            # qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            # qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            # qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)


            # pose_estimate.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

            # self.pose_publisher.publish(pose_estimate)
            
            rospy.loginfo("Robot model spawned successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to spawn robot model: %s", str(e))

if __name__ == '__main__':
    respawn_robot = RespawnRobot()
    respawn_robot.delete_and_spawn_robot()
