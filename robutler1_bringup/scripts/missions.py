#!/usr/bin/env python3

import rospy
import rospkg
import argparse
import os
import subprocess

def main():

    # -------------------------------
    # Initialization
    # -------------------------------

    # Argparse description and arguments
    parser = argparse.ArgumentParser(description='Script select missions and execute them.')
    parser.add_argument('-mid', '--mission_id', type=str,
                        help='Available missions by ID number: 1-Move To; 2-Take a Picture of Location; 3-Is Someone Home?; 4-Is Table Clean?; 5-Search for Object in Location,6-How Many Objects are in the House?; 7-Touch Object at Location',
                        required=False, default= '1')
    parser.add_argument('-m', '--mission', type=str,
                        help='Available missions by ID number: 1-Move To; 2-Take a Picture of Location; 3-Is Someone Home?; 4-Is Table Clean?; 5-Search for Object in Location,6-How Many Objects are in the House?; 7-Touch Object at Location',
                        required=False, default= 'Move To')

    args = vars(parser.parse_args())
    print(args)

    # -------------------------------
    # MISSIONS
    # -------------------------------

    # Missions dictionary
    missions = {
        '1': {'Task': 'Move To'},
        '2': {'Task': 'Take a Picture of Location'},
        '3': {'Task': 'Is Someone Home?'},
        '4': {'Task': 'Is Table Clean?'},
        '5': {'Task': 'Search for Object in Location'},
        '6': {'Task': 'How Many Objects are in the House?'},
        '7': {'Task': 'Touch Object at Location'}
    }

    mission_id = args['mission_id']
    Task = missions[mission_id]

    # 2 - 'Take a Picture of Location' mission 
    if mission_id in missions and mission_id == '2':
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('robutler1_bringup')  # Assuming 'robutler1_bringup' is the correct package name

        # Construct the path to the photo.py script
        script_path = os.path.join(package_path, 'photo.py')

        # Call the photo.py script using subprocess
        subprocess.call(['python3', script_path])

# -------------------------------
# MAIN
# -------------------------------

# Program execution starts here
if __name__ == '__main__':
    main()