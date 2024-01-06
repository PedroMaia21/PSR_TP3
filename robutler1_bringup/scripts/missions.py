#!/usr/bin/env python3

import rospy
import rospkg
import argparse
from spawn_object import object
from spawn_object import Pose

def main():

    # -------------------------------
    # Initialization
    # -------------------------------

    # Argparse description and arguments
    parser = argparse.ArgumentParser(description='Script select missions and execute them.')
    parser.add_argument('-m', '--missions', type=str,
                        help='Available missions: 1-Move To; 2-Take a Picture of Location; 3-Is Someone Home?; 4-Is Table Clean?; 5-Search for Object in Location,6-How Many Objects are in the House?; 7-Touch Object at Location',
                        required=False, default= '1')

    args = vars(parser.parse_args())              # creates a dictionary
    print(args)

    # -------------------------------
    # MISSIONS
    # -------------------------------
    missions = {}

    missions = {'1': 'Move To',
                '2': 'Take a Picture of Location',
                '3': 'Is Someone Home?',
                '4': 'Is Table Clean?',
                '5': 'Search for Object in Location',
                '6': 'How Many Objects are in the House?',
                '7': 'Touch Object at Location'}
    
    # For 'Take a Picture of Location' mission
    if missions['2']:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('robutler1_bringup') + '/photo.py/'
        
