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

    rospack = rospkg.RosPack()

    # 1 - 'Move To' mission 
    if mission_id in missions and mission_id == '1':
        package_path = rospack.get_path('robutler1_navigation')
        script_path = os.path.join(package_path, 'scripts/mission/mission_manager.py')

    # 2 - 'Take a Picture of Location' mission 
    if mission_id in missions and mission_id == '2':
        package_path = rospack.get_path('robutler1_bringup')
        script_path = os.path.join(package_path, 'scripts/photo.py')

    # To update once mission scripts are available
    # 3 - 'Is Someone Home?' mission 
    """ if mission_id in missions and mission_id == '3':
        package_path = rospack.get_path(' ')
        script_path = os.path.join(package_path, ' ')
    """

    # 4 - 'Is Table Clean?' mission 
    """ if mission_id in missions and mission_id == '4':
        package_path = rospack.get_path(' ')
        script_path = os.path.join(package_path, ' ')
    """

    # 5 - 'Search for Object in Location' mission 
    """ if mission_id in missions and mission_id == '5':
        package_path = rospack.get_path(' ')
        script_path = os.path.join(package_path, ' ')
    """

    # 6 - 'How Many Objects are in the House?' mission 
    """ if mission_id in missions and mission_id == '6':
        package_path = rospack.get_path(' ')
        script_path = os.path.join(package_path, ' ')
    """

    # 7 - 'Touch Object at Location' mission 
    """ if mission_id in missions and mission_id == '7':
        package_path = rospack.get_path(' ')
        script_path = os.path.join(package_path, ' ')
    """

    # Call the mission script using subprocess
    subprocess.call(['python3', script_path])

# -------------------------------
# MAIN
# -------------------------------

# Program execution starts here
if __name__ == '__main__':
    main()