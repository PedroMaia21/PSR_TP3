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

    # Argparse description and arguments
    parser = argparse.ArgumentParser(description='Script to spawn models in the house, in specific positions.')
    parser.add_argument('-l', '--location', type=str, 
                        help='Available locations are: on_bed; on_bed_side_table; on_desk; on_small_room; on_dinning_table; on_bedroom_floor; random_in_house;',
                        required=False, default='on_bed')
    parser.add_argument('-o', '--object', type=str,
                        help='Available objects are: bottle_red_wine; coca_cola_cube_b; human_female_1; human_male_1_1; laptop_pc_1; shere_r;',
                        required=False, default='sphere_r')

    args = vars(parser.parse_args())              # creates a dictionary
    print(args)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler1_description') + '/models/'

    # -------------------------------
    # Poses
    # -------------------------------

    """
    Defines poses where to put objects
    For the fixed poses, the coordinates are simply given through attribution
    For randome positions, in a certain area of the house, the coordinates are randomly generated, respecting the predefined limits
    The function "random.randrange()" only works with positive integers, therefor an offset and scale factor are necessary 
    The scale factor ensures that we only round the position value to the decimal place, not losing accuracy
    """
    
    poses = {}

    # on bed pose
    p = Pose()
    p.position = Point(x=-6.033466, y=1.971232, z=0.644345)
    q = quaternion_from_euler(0, 0, 0)              # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed'] = {'pose': p}                   # Adds position to poses list

    # on bed-side-table pose
    p = Pose()
    p.position = Point(x=-4.489786, y=2.867268, z=0.679033)
    q = quaternion_from_euler(0, 0, 0)              # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed_side_table'] = {'pose': p}

    # on desk pose
    p = Pose()
    p.position = Point(x=-9.0489, y=2.0433, z=0.7644)
    q = quaternion_from_euler(0, 0, 0)              # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_desk'] = {'pose': p}

    # on small room pose
    p = Pose()
    tempx = random.randrange(32,87)
    if tempx > 61:
        tempy = random.randrange(20,34)
    else:
        tempy = random.randrange(20,50)
    p.position = Point(x=-tempx/10, y=-tempy/10, z=0.3)
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
    tempy = random.randrange(0,11)                   # y from -0.90 to 0.20, we use a 10x factor and an offset (+9) to use the random.randrange function
    p.position = Point(x=-tempx/10, y=(tempy-9)/10, z=0.3)
    q = quaternion_from_euler(0, 0, tempY/100-3.14)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bedroom_floor'] = {'pose': p}

    # random in house pose
    p = Pose()                                       # this pose includes the biggest available zones in the whole house floor
    tempx = random.randrange(0,141)                  # x from -8.3 to 5.8, we use a 10x factor and an offset (+83)

    choosezone = random.randrange(0,1)               # choose a randome zone between two zones with the same x value
    choosezone1 = random.randrange(0,1)              # choose a randome zone between two zones with the same x value
    
    # multiple empty spots on the house are avaliable 
    if tempx <= 48:                                  # for x between -8,3 and -3,5

        if tempx >= 24 & choosezone == 0:
                tempy = random.randrange(15,16)      # for y between ymin -6.3 and ymax 2.6, we use a 10x factor and an offset (+63)
        else:
            tempy = random.randrange(54,65)

    elif tempx > 48:                                 # for x between -3,5 and 5,8
                                                   
        if choosezone == 0: 
            if tempx >= 92 & tempx <= 100 :
                if choosezone1 == 0:
                    tempy = random.randrange(18,39)
                else:
                    tempy = random.randrange(62,89)

            elif tempx >= 100 & tempx <= 137:
                tempy = random.randrange(18,39)

            elif tempx >= 68 & tempx <= 92:
                tempy = random.randrange(62,89)

        else:
            tempy = random.randrange(0,10)

    p.position = Point(x=(tempx-83)/10, y=(tempy-63)/10, z=0.5)
    q = quaternion_from_euler(0, 0, tempY/100-3.14)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['random_in_house'] = {'pose': p}

    # -------------------------------
    # Objects
    # -------------------------------

    objects = {}

    # add object sphere_r
    f = open(package_path + 'sphere_r/model.sdf', 'r')
    objects['sphere_r'] = {'name': 'sphere_r', 'sdf': f.read()}

    # add object bottle_red_wine
    f = open(package_path + 'bottle_red_wine/model.sdf', 'r')
    objects['bottle_red_wine'] = {'name': 'bottle_red_wine', 'sdf': f.read()}

    # add object coca_cola
    f = open(package_path + 'coca_cola/model.sdf', 'r')
    objects['coca_cola'] = {'name': 'coca_cola', 'sdf': f.read()}

    # add object cube_b
    f = open(package_path + 'cube_b/model.sdf', 'r')
    objects['cube_b'] = {'name': 'cube_b', 'sdf': f.read()}

    # add object human_female_1
    f = open(package_path + 'human_female_1/model.sdf', 'r')
    objects['human_female_1'] = {'name': 'human_female_1', 'sdf': f.read()}

    # add object human_male_1_1
    f = open(package_path + 'human_male_1_1/model.sdf', 'r')
    objects['human_male_1_1'] = {'name': 'human_male_1_1', 'sdf': f.read()}

    # add object laptop_pc_1
    f = open(package_path + 'laptop_pc_1/model.sdf', 'r')
    objects['laptop_pc_1'] = {'name': 'laptop_pc_1', 'sdf': f.read()}


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


# -------------------------------
# MAIN
# -------------------------------

if __name__ == '__main__':
    main()