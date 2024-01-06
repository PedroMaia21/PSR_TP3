#!/usr/bin/env python3

import rospy
import rospkg
import argparse
from missions import mission 

def main():

    # -------------------------------
    # Initialization
    # -------------------------------

    # Argparse description and arguments
    parser = argparse.ArgumentParser(description='Script take photo of a location.')
    parser.add_argument('-l', '--location', type=str, help='', required=False, default= '')
    parser.add_argument('-o', '--object', type=str, help='', required=False, default= '')

    args = vars(parser.parse_args())              # creates a dictionary
    print(args)

    # -------------------------------
    # MISSIONS
    # -------------------------------