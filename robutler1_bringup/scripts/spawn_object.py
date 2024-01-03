#!/usr/bin/env python3

import random

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import uuid
import argparse


def main():

    # -------------------------------
    # Initialization
    # -------------------------------
    parser = argparse.ArgumentParser(description='Script to spawn models in confined areas, but random positions.')
    parser.add_argument('-l', '--location', type=str, help='', required=False,
                        default='on_bed')
    parser.add_argument('-o', '--object', type=str, help='', required=False,
                        default='sphere_r')

    args = vars(parser.parse_args())  # creates a dictionary
    print(args)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler1_description') + '/models/'

    # Defines poses where to put objects
    poses = {}

    # on bed pose
    p = Pose()
    p.position = Point(x=-6.033466, y=1.971232, z=0.644345)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed'] = {'pose': p}

    # on bed-side-table pose
    p = Pose()
    p.position = Point(x=-4.489786, y=2.867268, z=0.679033)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed_side_table'] = {'pose': p}

    # on desk pose
    p = Pose()
    p.position = Point(x=-9.0489, y=2.0433, z=0.7644)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_desk'] = {'pose': p}

    # on small room pose
    p = Pose()
    tempx = random.randrange(32,87)
    if tempx > 61:
        tempy = random.randrange(20,34)
    else:
        tempy = random.randrange(20,50)
    p.position = Point(x=-tempx/10, y=-tempy/10, z=0.5)
    tempY = random.randrange(0, 628)
    q = quaternion_from_euler(0, 0, tempY/100-3.14)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_small_room'] = {'pose': p}

    # on dinning table pose
    p = Pose()
    tempx = random.randrange(56,74)
    tempy = random.randrange(10,13)
    p.position = Point(x=tempx/10, y=tempy/10, z=0.817812)
    q = quaternion_from_euler(0, 0, tempY/100-3.14)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_dinning_table'] = {'pose': p}

    # on bedroom floor pose
    p = Pose()
    tempx = random.randrange(35,83)
    tempy = random.randrange(-8.6,2.4)
    p.position = Point(x=-tempx/10, y=tempy/10, z=0.5)
    q = quaternion_from_euler(0, 0, tempY/100-3.14)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bedroom_floor'] = {'pose': p}

    # random in house pose
       
    p = Pose()                                       # this pose includes the biggest available zones in the whole house floor
    tempx = random.randrange(-83,58)

    choosezone = random.randrange(0,1)               # choose a randome zone between two zones with the same x value
    choosezone1 = random.randrange(0,1)               # choose a randome zone between two zones with the same x value
    
    if tempx <= -35.5:                               # for x bettewn -8,3 and -3,5

        if tempx >= -59 & choosezone == 0:
                tempy = random.randrange(-47.5,-46.5)
        else:
            tempy = random.randrange(-8.6,2.4)

    elif tempx > -35.5:                              # for x bettewn -3,5 and 5,8
                                                   
        if choosezone == 0: 
            if tempx >= 9 & tempx <= 16.9:
                if choosezone1 == 0:
                    tempy = random.randrange(-45,-24)
                else:
                    tempy = random.randrange(-0.5,25.7)

            elif tempx >= 16.9 & tempx <= 54:
                tempy = random.randrange(-45,-24)

            elif tempx >= -14.5 & tempx <= 9:
                tempy = random.randrange(-0.5,25.7)

        else:
            tempy = random.randrange(-62.9,-53)

    p.position = Point(x=tempx/10, y=tempy/10, z=0.5)
    q = quaternion_from_euler(0, 0, tempY/100-3.14)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['random_in_house'] = {'pose': p}

    # define objects
    objects = {}

    # add object sphere_r
    f = open(package_path + 'sphere_r/model.sdf', 'r')
    objects['sphere_r'] = {'name': 'sphere_r', 'sdf': f.read()}

    # add object sphere_r
    # f = open(package_path + 'sphere_r/model.sdf', 'r')
    # objects['sphere_r'] = {'name': 'sphere_r', 'sdf': f.read()}

    # Check if given object and location are valid

    if not args['location'] in poses.keys():
        print('Location ' + args['location'] +
              ' is unknown. Available locations are ' + str(list(poses.keys())))

    if not args['object'] in objects.keys():
        print('Object ' + args['object'] +
              ' is unknown. Available objects are ' + str(list(objects.keys())))

    # -------------------------------
    # ROS
    # -------------------------------

    rospy.init_node('insert_object', log_level=rospy.INFO)

    service_name = 'gazebo/spawn_sdf_model'
    print('waiting for service ' + service_name + ' ... ', end='')
    rospy.wait_for_service(service_name)
    print('Found')

    service_client = rospy.ServiceProxy(service_name, SpawnModel)

    print('Spawning an object ...')
    uuid_str = str(uuid.uuid4())
    service_client(objects[args['object']]['name'] + '_' + uuid_str,
                   objects[args['object']]['sdf'],
                   objects[args['object']]['name'] + '_' + uuid_str,
                   poses[args['location']]['pose'],
                   'world')

    print('Done')


if __name__ == '__main__':
    main()