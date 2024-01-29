#!/usr/bin/env python3

import rospy
import tkinter as tk
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from subprocess import Popen
import rospkg
import time
from mission.mission_manager import moveTo, lookFor

class RobutlerUI:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robutler_ui', anonymous=True)

        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)    

        rospack = rospkg.RosPack()
        bringup_package_path = rospack.get_path('robutler1_bringup')
        navigation_package_path = rospack.get_path('robutler1_navigation')

        self.spawn_script_path = f'{bringup_package_path}/scripts/spawn_object.py'
        self.clear_script_path = f'{bringup_package_path}/scripts/clear_spawned_objects.py'
        self.respawn_script_path = f'{bringup_package_path}/scripts/respawn_robot.py'
        self.picture_script_path = f'{bringup_package_path}/scripts/photo.py'
        self.teleop_script_path = f'{navigation_package_path}/scripts/teleop/new_teleop_key.py'

        # Create a Tkinter window
        self.root = tk.Tk()
        self.root.title("Robutler Simulation Control")

        # Define GUI components
        self.selected_object = tk.StringVar()
        self.selected_location = tk.StringVar()
        self.spawn_quantity_var = tk.StringVar(value='1')  # Default spawn quantity is 1

        object_label = tk.Label(self.root, text="Select Object:")
        location_label = tk.Label(self.root, text="Select Location:")
        quantity_label = tk.Label(self.root, text="Spawn Quantity:")

        object_radio_frame = tk.Frame(self.root)
        location_radio_frame = tk.Frame(self.root)

        object_radio1 = tk.Radiobutton(object_radio_frame, text="red ball", variable=self.selected_object, value="sphere_r")
        object_radio2 = tk.Radiobutton(object_radio_frame, text="bottle", variable=self.selected_object, value="bottle_red_wine")
        object_radio3 = tk.Radiobutton(object_radio_frame, text="can", variable=self.selected_object, value="coca_cola")
        object_radio4 = tk.Radiobutton(object_radio_frame, text="blue cube", variable=self.selected_object, value="cube_b")
        object_radio5 = tk.Radiobutton(object_radio_frame, text="woman", variable=self.selected_object, value="human_female_4")
        object_radio6 = tk.Radiobutton(object_radio_frame, text="man", variable=self.selected_object, value="human_male_4")
        object_radio7 = tk.Radiobutton(object_radio_frame, text="laptop", variable=self.selected_object, value="laptop_pc_1")

        location_radio1 = tk.Radiobutton(location_radio_frame, text="bed", variable=self.selected_location, value="on_bed")
        location_radio2 = tk.Radiobutton(location_radio_frame, text="bed side table", variable=self.selected_location, value="on_bed_side_table")
        location_radio3 = tk.Radiobutton(location_radio_frame, text="desk", variable=self.selected_location, value="on_desk")
        location_radio4 = tk.Radiobutton(location_radio_frame, text="small room", variable=self.selected_location, value="on_small_room")
        location_radio5 = tk.Radiobutton(location_radio_frame, text="dinning table", variable=self.selected_location, value="on_dinning_table")
        location_radio6 = tk.Radiobutton(location_radio_frame, text="bedroom", variable=self.selected_location, value="on_bedroom_floor")
        location_radio7 = tk.Radiobutton(location_radio_frame, text="random", variable=self.selected_location, value="random_in_house")

        quantity_entry = tk.Entry(self.root, textvariable=self.spawn_quantity_var, width=5)

        spawn_button = tk.Button(self.root, text="Spawn Object", command=self.spawn_object)
        clear_button = tk.Button(self.root, text="Clear Objects", command=self.clear_objects)
        move_button = tk.Button(self.root, text="Move to location", command=self.move_to_location)
        look_button = tk.Button(self.root, text="Look for object in location", command=self.look_for_object)
        respawn_button = tk.Button(self.root, text="Respawn Robot", command=self.respawn_robot)
        teleop_button = tk.Button(self.root, text="Start Teleop", command=self.start_teleop)
        picture_button = tk.Button(self.root, text="Take Picture", command=self.take_picture)

        # Arrange GUI components
        object_label.pack(pady=5)
        object_radio_frame.pack(pady=5)
        object_radio1.pack(side=tk.LEFT, anchor=tk.W)
        object_radio2.pack(side=tk.LEFT, anchor=tk.W)
        object_radio3.pack(side=tk.LEFT, anchor=tk.W)
        object_radio4.pack(side=tk.LEFT, anchor=tk.W)
        object_radio5.pack(side=tk.LEFT, anchor=tk.W)
        object_radio6.pack(side=tk.LEFT, anchor=tk.W)
        object_radio7.pack(side=tk.LEFT, anchor=tk.W)

        location_label.pack(pady=5)
        location_radio_frame.pack(pady=5)
        location_radio1.pack(side=tk.LEFT, anchor=tk.W)
        location_radio2.pack(side=tk.LEFT, anchor=tk.W)
        location_radio3.pack(side=tk.LEFT, anchor=tk.W)
        location_radio4.pack(side=tk.LEFT, anchor=tk.W)
        location_radio5.pack(side=tk.LEFT, anchor=tk.W)
        location_radio6.pack(side=tk.LEFT, anchor=tk.W)
        location_radio7.pack(side=tk.LEFT, anchor=tk.W)

        quantity_label.pack(pady=5)
        quantity_entry.pack()

        spawn_button.pack(pady=10)
        clear_button.pack(pady=10)
        move_button.pack(pady=10)
        look_button.pack(pady=10)
        respawn_button.pack(pady=10)
        teleop_button.pack(pady=10)
        picture_button.pack(pady=10)

    def spawn_object(self):
        # Get the selected object, location, and spawn quantity
        selected_object = self.selected_object.get()
        selected_location = self.selected_location.get()
        spawn_quantity = self.spawn_quantity_var.get()

        # Call the spawn_object.py script with the selected arguments
        for i in range(int(spawn_quantity)):
            spawn_command = ['python3', self.spawn_script_path, '--object', selected_object, '--location', selected_location]

            try:
                Popen(spawn_command)
                rospy.loginfo("Spawn command executed successfully.")
            except Exception as e:
                rospy.logerr("Error executing spawn command: %s", str(e))
            time.sleep(5)

    def clear_objects(self):
        # Call a service to clear all spawned objects
        clear_command = ['python3', self.clear_script_path]

        try:
            Popen(clear_command)
            rospy.loginfo("Clear command executed successfully.")
        except Exception as e:
            rospy.logerr("Error executing clear command: %s", str(e))

    def respawn_robot(self):
        # Call a service to respawn the robot
        respawn_command = ['python3', self.respawn_script_path]

        try:
            Popen(respawn_command)
            rospy.loginfo("Respawn command executed successfully.")
        except Exception as e:
            rospy.logerr("Error executing respawn command: %s", str(e))

    def start_teleop(self):
        # Implement the logic to start the teleoperation script
        teleop_command = ['python3', self.teleop_script_path]

        try:
            Popen(teleop_command)
        except Exception as e:
            rospy.logerr("Error executing teleop command: %s", str(e))

    def take_picture(self):
        # Implement the logic to start the teleoperation script
        picture_command = ['python3', self.picture_script_path]

        try:
            Popen(picture_command)
        except Exception as e:
            rospy.logerr("Error executing take picture command: %s", str(e))
    
    def move_to_location(self):
        selected_location = self.selected_location.get()

        #problems with comaptibility between scripts
        if selected_location == 'on_bed':
            selected_location = 'bed'
        if selected_location == 'on_bed_side_table':
            selected_location = 'bed side table'
        if selected_location == 'on_desk':
            selected_location = 'desk'
        if selected_location == 'on_small_room':
            selected_location = 'small room'
        if selected_location == 'on_dinnig_table':
            selected_location = 'dinning table'
        if selected_location == 'on_bedroom_floor':
            selected_location = 'bedroom'
        if selected_location == 'random_in_house':
            selected_location = 'house'

        moveTo(feedback="Sub_process", location=selected_location, goal_publisher=self.goal_publisher)

    def look_for_object(self):
        selected_object = self.selected_object.get()
        if selected_object == 'sphere_v':
            selected_object = 'red ball'
        if selected_object == 'bottle_red_wine':
            selected_object = 'bottle'
        if selected_object == 'coca_cola':
            selected_object = 'can'
        if selected_object == 'cube_b':
            selected_object = 'blue cube'
        if selected_object == 'human_female_4':
            selected_object = 'woman'
        if selected_object == 'human_male_4':
            selected_object = 'man'
        if selected_object == 'laptop_pc_1':
            selected_object = 'laptop'
        
        selected_location = self.selected_location.get()
        #problems with comaptibility between scripts
        if selected_location == 'on_bed':
            selected_location = 'bed'
        if selected_location == 'on_bed_side_table':
            selected_location = 'bed side table'
        if selected_location == 'on_desk':
            selected_location = 'desk'
        if selected_location == 'on_small_room':
            selected_location = 'small room'
        if selected_location == 'on_dinnig_table':
            selected_location = 'dinning table'
        if selected_location == 'on_bedroom_floor':
            selected_location = 'bedroom'
        if selected_location == 'random_in_house':
            selected_location = 'house'
        lookFor(feedback="Sub_process", object=selected_object, spawn_location=selected_location, goal_publisher=self.goal_publisher)

    def run(self):
        # Run the Tkinter main loop
        self.root.mainloop()        

if __name__ == '__main__':
    robutler_ui = RobutlerUI()
    robutler_ui.run()
