#!/usr/bin/env python3

from functools import partial
import rospy
import rospkg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import os
import subprocess


from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult

server = None
marker_pos = 1

menu_handler = MenuHandler()

h_first_entry = 0
h_second_entry = 0
h_mode_last = 0

rospack = rospkg.RosPack()

locations = {
    'kitchen': {'x': 6.568593, 'y': -1.788789, 'z': 0, 'R': 0, 'P': 0, 'Y': -1.504141},
    'bedroom': {'x': -6.172946, 'y': -0.411477, 'z': 0, 'R': 0, 'P': 0, 'Y': 1.57},
    'small room': {'x': -5.334507, 'y': -3.049341, 'z': 0, 'R': 0, 'P': 0, 'Y': -2.632790},
    'living room': {'x': 1.124490, 'y': -0.047983, 'z': 0, 'R': 0, 'P': 0, 'Y': -1.57},
    'gym': {'x': 1.650620, 'y': 4.189130, 'z': 0, 'R': 0, 'P': 0, 'Y': -0.865260},
    'next to desk': {'x': -7.899468, 'y': 0.981056, 'z': 0, 'R': 0, 'P': 0, 'Y': 2.489528},
}

spawn_locations = {
    'bed': 'on_bed',
    'bed side table': 'on_bed_side_table',
    'desk': 'on_desk',
    'small room': 'on_small_room',
    'dinning table': 'on_dinning_table',
    'bedroom': 'on_bedroom_floor',
    'house': 'random_in_house'
}

objects = {
    'red ball': 'sphere_r', 
    'bottle': 'bottle_red_wine',
    'can': 'coca_cola',
    'blue cube': 'cube_b',
    'woman': 'human_female_1',
    'man': 'human_male_1_1',
    'laptop': 'laptop_pc_1'
}

def enableCb(feedback):
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState(handle)

    if state == MenuHandler.CHECKED:
        menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
        rospy.loginfo("Hiding first menu entry")
        menu_handler.setVisible(h_first_entry, False)
    else:
        menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        rospy.loginfo("Showing first menu entry")
        menu_handler.setVisible(h_first_entry, True)

    menu_handler.reApply(server)
    rospy.loginfo("update")
    server.applyChanges()


def modeCb(feedback):
    global h_mode_last
    menu_handler.setCheckState(h_mode_last, MenuHandler.UNCHECKED)
    h_mode_last = feedback.menu_entry_id
    menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)

    rospy.loginfo("Switching to menu entry #" + str(h_mode_last))
    menu_handler.reApply(server)
    print("DONE")
    server.applyChanges()


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.2

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def makeEmptyMarker(dummyBox=True):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position.z = marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker


def makeMenuMarker(name):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append(makeBox(int_marker))
    int_marker.controls.append(control)

    server.insert(int_marker)


def deepCb(feedback):
    rospy.loginfo("The deep sub-menu has been found.")


def moveTo(location, goal_publisher):

    print('Called moving to ' + location)
    coordinates = locations.get(location)
    p = Pose()
    p.position = Point(x=coordinates['x'], y=coordinates['y'], z=coordinates['z'])
    q = quaternion_from_euler(coordinates['R'], coordinates['P'], coordinates['Y'])  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    ps = PoseStamped()
    ps.pose = p
    ps.header = Header(frame_id='map', stamp=rospy.Time.now())

    print('Sending Goal move to ' + location)
    goal_publisher.publish(ps)

    # TODO know when move is finished

    try:
        result_msg = rospy.wait_for_message('/move_base/result', MoveBaseActionResult, timeout=360)
    except:
        print('Timeout waiting for moveto')
        # TODO
        return

    print('move base completed goal with result ' + str(result_msg))

def LookFor(feedback, object, spawn_location, goal_publisher):
    
    print('Called looking for ' + object + ' in ' + spawn_location)

    #spawn the object first
    spawn_name = objects.get(object)
    spawn_location_name = spawn_locations.get(spawn_location)
    package_path = rospack.get_path('robutler1_bringup')
    script_path = os.path.join(package_path, 'scripts/spawn_object.py')
    spawn_arguments = ['--location', spawn_location_name, '--object', spawn_name]
    
    command = ['python3', script_path] + spawn_arguments

    subprocess.call(command)

    #then go there
    if spawn_location == 'bed' or spawn_location == 'bed side table':
        location = 'bedroom'
    elif spawn_location == 'desk':
        location = 'next to desk'
    elif spawn_location == 'dinning table':
        location = 'kitchen'
    elif spawn_location == 'house':
        location = 'living room'    #needs to be changed
    else:
        location = spawn_location

    moveTo(location=location, goal_publisher=goal_publisher)

    #lastly it checks

def main():

    global server

    # -------------------------------
    # Initialization
    # -------------------------------
    rospy.init_node("mission_manager")

    # Create move_base_simple/goal publisher
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    server = InteractiveMarkerServer("mission")
    print(server)

    global h_first_entry, h_second_entry, h_mode_last
    h_first_entry = menu_handler.insert("Move to")

    entry = menu_handler.insert("kitchen", parent=h_first_entry,
                                callback=partial(moveTo, location='kitchen', goal_publisher=goal_publisher))

    entry = menu_handler.insert("bedroom", parent=h_first_entry,
                                callback=partial(moveTo, location='bedroom', goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("small room", parent=h_first_entry,
                                callback=partial(moveTo, location='small room', goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("living room", parent=h_first_entry,
                                callback=partial(moveTo, location='living room', goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("gym", parent=h_first_entry,
                                callback=partial(moveTo, location='gym', goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("next to desk", parent=h_first_entry,
                                callback=partial(moveTo, location='next to the desk', goal_publisher=goal_publisher))
    
    h_second_entry = menu_handler.insert("Look for")
    sub_handler1 = menu_handler.insert("red ball", parent=h_second_entry)
    entry = menu_handler.insert("in small room", parent=sub_handler1,
                                callback=partial(LookFor, object='red ball', spawn_location='small room',goal_publisher=goal_publisher))
                                
    makeMenuMarker("marker1")

    menu_handler.apply(server, "marker1")
    server.applyChanges()

    rospy.spin()


# -------------------------------
# MAIN
# -------------------------------

if __name__ == '__main__':
    main()