#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios
import tkinter as tk

BURGER_MAX_LIN_VEL = 2.2
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 2.6
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)

    return vel

class TeleopApp:
    def __init__(self, master):
        self.master = master
        self.master.title("TurtleBot3 Teleop")
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0

        # Create a label to display the current velocities
        self.vel_label = tk.Label(master, text=vels(self.target_linear_vel, self.target_angular_vel))
        self.vel_label.pack()

        # Bind arrow key events to the corresponding functions
        master.bind('<Up>', self.increase_linear_vel)
        master.bind('<Down>', self.decrease_linear_vel)
        master.bind('<Left>', self.increase_angular_vel)
        master.bind('<Right>', self.decrease_angular_vel)

        # Bind space key and 's' key to stop
        master.bind('<space>', self.stop)
        master.bind('s', self.stop)

        # Bind 'm' key to stop angular velocity
        master.bind('m', self.stop_angular)

        # Update the label with current velocities
        self.update_label()

    def increase_linear_vel(self, event):
        self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
        self.update_label()

    def decrease_linear_vel(self, event):
        self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
        self.update_label()

    def increase_angular_vel(self, event):
        self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
        self.update_label()

    def decrease_angular_vel(self, event):
        self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
        self.update_label()

    def stop(self, event):
        self.target_linear_vel = 0.0
        self.control_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_angular_vel = 0.0
        self.update_label()

    def stop_angular(self, event):
        self.target_angular_vel = 0.0
        self.control_angular_vel = 0.0
        self.update_label()

    def update_label(self):
        self.vel_label.config(text=vels(self.target_linear_vel, self.target_angular_vel))
        twist = Twist()
        control_linear_vel = makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
        pub.publish(twist)

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "waffle_pi")

    try:
        root = tk.Tk()
        app = TeleopApp(root)
        root.mainloop()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
