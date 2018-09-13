#Author: Benjamin Hilton
#ME 537 Project
#Date: December 2017
#Title: Baxter_client_ME537


#system level imports
import sys, os
import numpy as np
import scipy.io as sio
import socket
import time
import math #for sqrt function

#libraries for baxter
from math import pi
from baxter_interface.limb import Limb
from rad_baxter_limb import RadBaxterLimb
from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf


#This function receives the bbox (bounding box) info from the server computer (Baxter_server_ME537). Calculates the error given COFx and COFy (center of frames, set below)
def getError():
    my_string = s.recv(15)
    #The server computer sends strings in format (###-###-###-###) where left is the x pixel value of the left side of the bbox, top is the y value of the top of the bbox, and width and height are the width and height of the bbox
    left, top, width, height = my_string.split("-")
    x_error = int(left) + (int(width) / 2) - COFx
    y_error = int(top) + (int(height) / 2) - COFy
    mag_of_error = math.sqrt(x_error**2 + y_error**2)
    return_list = [x_error, y_error, mag_of_error]
    return return_list

#Calculates the current pose
def getCurrentPose():
    my_list = limb.get_kdl_forward_position_kinematics()
    pos = my_list[0:3]
    orientation = my_list[3:len(my_list)]
    return pos, orientation


#Setup socket stuff - socket was used because Baxter's computer did not have the necessary opencv libraries.  It would need to be reinstalled.  Rather than risk messing up other code on the Baxter computer, we opted to separate the opencv code and the Baxter code and pass the information over the network. As long as both computers are on the same network, this will work.  In the case of the ME 537 Project, the BYU Secure network was used. This also allowed us to develop the programs separately.
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '10.4.108.209' #this needs to be changed to match IP of server computer
port = 9547 #chosen arbitrarily, as long as not used for other service. Must match the port number in
s.connect((host, port))



#Initialize
rospy.init_node('ME537_Project')

#Specify which arm we are using
limb = RadBaxterLimb('right')

#Set the max speed
limb.set_joint_position_speed(0.1)

#This defines how long we wait to send a new command in Hz
control_rate = rospy.Rate(200)

#Record initial position
start_joint_command = limb.get_joint_angles()

#Calibrate the z position down to where it is pressing a key.
raw_input("move to your desired ending position and push enter ...")
z_list = limb.get_kdl_forward_position_kinematics()
z_pos = z_list[0:3]
z_key_press = z_list[2]

#From where Baxter was (at z key press) move back up to starting position
time_to_wait = 5
step = 1
while step < time_to_wait*500:
    limb.set_joint_positions_mod(start_joint_command)
    control_rate.sleep()
    step = step + 1

#Set the speed lower - too fast will shake the camera and the tracker will fail.
limb.set_joint_position_speed(0.01)

#Get forward position kinematics, Separate into position and orientation
my_list = limb.get_kdl_forward_position_kinematics()
initial_pos = my_list[0:3]
initial_orientation = my_list[3:len(my_list)]
desired_ori = initial_orientation

#Center of Frame - obtained empirically
COFx = 321
COFy = 248

#Set gain for error terms
gain = 0.0003


if __name__ == '__main__':


    while not rospy.is_shutdown(): #run until the shutdown (or until error because server code has finished)

        while True:

            #this loop will loop until the key is centered in the frame (no change in z)
            values = getError() #gets x_error, y_error, and magnitude of error from function
            desired_pos, current_O = getCurrentPose() #gets current pose from function
            desired_pos[0] = desired_pos[0] + (values[0] * gain) #set desired to (current + error*gain) for x direction
            desired_pos[1] = desired_pos[1] - (values[1] * gain) #set desired to (current - error*gain) for y direction, it is negative because the pixel y axis and the Baxter y axis are in opposite directions.

            #Get current joint values, for seeding the IK algorithm
            seed = limb.get_joint_angles()

            #Convert arrays into lists to use in IK
            desired_pos = desired_pos.tolist()
            desired_ori = initial_orientation.tolist()
            seed = seed.tolist()

            #Calculate IK
            q_des = limb.kin_kdl.inverse_kinematics(desired_pos, desired_ori, seed)

            #Move the arm to the position specified in q_des
            limb.set_joint_positions_mod(q_des)

		#if the error magnitude is less than 10 pixels, break
            if (values[2] < 10):
                break

            control_rate.sleep()



        while True:
            #this loop will loop until the z is at a certain threshold

            values = getError() #gets x_error, y_error, and magnitude of error from function
            current_pos, current_O = getCurrentPose() #gets current pose from function

            desired_pos[0] = current_pos[0] + (values[0] * gain) #set desired to (current + error*gain) for x direction
            desired_pos[1] = current_pos[1] - (values[1] * gain) #set desired to (current - error*gain) for y direction
            desired_pos[2] = current_pos[2] - 0.005 #set desired to (current - 0.005) for z direction

            #Get current joint values, for seeding the IK algorithm
            seed = limb.get_joint_angles()

            #Convert arrays into lists to use in IK
            desired_ori = initial_orientation.tolist()
            seed = seed.tolist()

            #Calculate IK
            q_des = limb.kin_kdl.inverse_kinematics(desired_pos, desired_ori, seed)

            #Move the arm to the position specified in q_des
            limb.set_joint_positions_mod(q_des)

		#move down until the camera is 8.5 cm above the key. Much lower and the image will blur and shake and the tracker will fail
            if (current_pos[2] < z_key_press + 0.085):
                break

            control_rate.sleep()



        time_to_wait = 5
        step = 1
        current_pos, current_O = getCurrentPose()

	  #Set desired position to the current x and y position and the z required to press a key.
        desired_pos = current_pos
        desired_pos[0] = desired_pos[0] + 0.002 #offset of 2 mm (obtained by trial and error)
        desired_pos[1] = desired_pos[1] - 0.002 #offset of 2 mm (obtained by trial and error)
        desired_pos[2] = z_key_press
        seed = limb.get_joint_angles()

	  #Convert arrays into lists to use in IK
        desired_pos = desired_pos.tolist()
        current_O = current_O.tolist()
        seed = seed.tolist()

	  #Calculate IK
        q_des = limb.kin_kdl.inverse_kinematics(desired_pos, current_O, seed)

	  #Move faster
        limb.set_joint_position_speed(0.1)

        #Move down to press the key
        while step < time_to_wait*52: #value chosen by trial and error - gives the right speed to only hit key once
            limb.set_joint_positions_mod(q_des)
            control_rate.sleep()
            step = step + 1

        #Return back up
        step = 1
        while step < time_to_wait*500:
            limb.set_joint_positions_mod(seed)
            control_rate.sleep()
            step = step + 1

        #Return to starting position
        step = 1
        while step < time_to_wait*650: #give extra time to ensure that arrives
            limb.set_joint_positions_mod(start_joint_command)
            control_rate.sleep()
            step = step + 1

	  #Prompt the user to continue to the next letter
        raw_input("Push enter to go on to next letter.")

	  #Set joint speed to slow again
        limb.set_joint_position_speed(0.01)
