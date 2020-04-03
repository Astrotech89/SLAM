"""
"""

from __future__ import print_function
from src.env    import VrepEnvironment
from src.agents import Pioneer
from src.disp   import Display
import settings
import time, argparse
import matplotlib.pyplot as plt
import numpy as np

""" Motors:

		  1. agent.change_velocity([ speed_left: float, speed_right: float ]) 
			   Set the target angular velocities of left
			   and right motors with a LIST of values:
			   e.g. [1., 1.] in radians/s.
			   
			   Values in range [-5:5] (above these 
			   values the control accuracy decreases)
					
		  2. agent.current_velocity() 
		  ----
			   Returns a LIST of current angular velocities
			   of the motors
			   [speed_left: float, speed_right: float] in radians/s.

	Lidar:
		  3. agent.read_lidar()   
		  ----
			   Returns a list of floating point numbers that you can 
			   indicate the distance towards the closest object at a particular angle.
			   
			   Basic configuration of the lidar:
			   Angle: [-int(int(data_length/2)):int(int(data_length/2))] Starting with the 
			   leftmost lidar point -> clockwise

	Agent:
		  You can access these attributes to get information about the agent's positions

		  4. agent.pos  

		  ----
			   Current x,y position of the agent (derived from 
			   SLAM data)

		  5. agent.position_history

			   A deque containing N last positions of the agent 
			   (200 by default, can be changed in settings.py)
"""

def got_stuck_spin(agent):
	'''
	If called will spin the robot and find the longest free path and move in that direction 
	for a set length of time.  May be unstable
	'''

	data = agent.read_lidars()
	agent.change_velocity([-0.5,0.5])

	# if data[int(int(data_length/2))] == max(data):
	# 	agent.change_velocity([1,1])
	# 	time.sleep(3)
	# 	agent.change_velocity([0,0])
	# 	time.sleep(3)


def reverse(agent):
	'''
	Simply reverses the bot then rotates it
	'''

	agent.change_velocity([-1,-1])
	time.sleep(3)
	agent.change_velocity([1,-1])
	time.sleep(1)
	agent.change_velocity([0,0])

# def pos_hist(agent):
# 	agent.position_history() # Need something with this to help get unstuck
# 	# yeah not sure what this is doing yet, didnt need it until now


# global counter
# counter = 1

	
def loop(agent):
	count_circle = 980000/5 #amount of counts for a full circle
	
	# numbers that worked:
	# lower_threshold = 2
	# upper_threshold = 9
	# cone_size = 45
	# agent.change_velocity([0.2,-0.2])
	# time.sleep(2.15)


	# numbers that worked:
	# lower_threshold = 2
	# upper_threshold = 12
	# cone_size = 50
	# agent.change_velocity([0.42,-0.42])
	# time.sleep(0) # no time to sleep


	flag_rotate = True
	flag_move = False
	pass_through_door_flag = False
	cone_size = 50 # angular size of the cone in degrees
	lower_threshold = 2 # lower threshold for the distance difference between the center and the edge of the cone 
	upper_threshold = 12 # lower threshold for the distance difference between the center and the edge of the cone 
	counter = 1

	data = agent.read_lidars()
	print(data[int(len(data)/2)])
	data = agent.read_lidars()[45: -45]
	print(data[int(len(data)/2)])
	data_length = len(data)
	print(data_length)

	
	print ("Searching for target\nBZZZ")
	agent.change_velocity([0.25,-0.25]) # start rotating

	while flag_rotate: #While condition is true

		counter += 1
		data = agent.read_lidars()
		
		# Find the indices of the center (maximum value of the data) and the edges of the cone
		max_distance_index = data.index(max(data))
		upper_boundary_index = max_distance_index + cone_size
		lower_boundary_index = max_distance_index - cone_size

		
		if min(data) < 0.35:
			flag_rotate = False
			flag_move = True
			print("HALP! Im stuck between doors")

		# Find the distance for the center and the edges of the cone
		if lower_boundary_index>0 and upper_boundary_index < data_length:
			left_distance = data[lower_boundary_index]
			max_distance = data[max_distance_index]
			right_distance = data[upper_boundary_index]

			# if the maximum distance is at the center of the FOV and the distance difference between the center and the edges of the cone are within the thresholds 
			# if 133 < max_distance_index < 137:
			if max_distance_index == int(int(data_length/2)):
				if lower_threshold < max_distance-right_distance < upper_threshold:
					if lower_threshold < max_distance-left_distance < upper_threshold:
					
						print("Target Locked")
						# time.sleep(0.6)
						agent.change_velocity([0,0])
						# print("left edge of cone index: ", data.index(data[lower_boundary_index]))
						# print("center of the cone index: ", data.index(data[max_distance_index]))
						# print("right edge of cone index: ", data.index(data[upper_boundary_index]))
						
						flag_rotate = False # stop the robot from rotating
						flag_move = True # start the next while

		#If robot has completed a full circle without moving.  Counts determined by experimentation
		if counter >= count_circle:
			print('No Detection, im dizzy.\nLets wiggle!')
			# got_stuck_spin(agent)
			counter = 1 #Resets counter
			flag_move = True #Resets loop
			flag_rotate = False #Same
	# print(counter)
	
	
	agent.change_velocity([7,7])
	counter = 1
	# middle_step = 0

	#While tuple is true
	# The robot starts moving forward until it sees an obstacle at a distance less than 2 units directly in front of it 
	while flag_move:
		counter += 1
		data = agent.read_lidars()
		max_distance_index = data.index(max(data))
		max_distance = data[max_distance_index]

		
			

		# # Hit wall condition
		# if data[int(int(data_length/2))] < 0.5:
		# 	print('Hit a Wall.\n AAAH PANIC!!!')
		# 	got_stuck_spin(agent)
		# 	flag_move = False # Stops the robot

		# If it gets stuck for too long without an obstacle directly in front of it
		if counter >= 100000:
			print('Nothing to see here, Go Back!')
			reverse(agent)
			flag_move = False # Stops the robot

		# Hit wall condition
		if min(data) < 0.33:
			# print("stuck in wall")
			min_distance_index = data.index(min(data))
			min_distance = data[min_distance_index]

			if int(int(data_length/4)) < min_distance_index < int(int(data_length/2)):
				print("stuck in wall")
				agent.change_velocity([-1,-1])
				time.sleep(3)
				agent.change_velocity([-1,1])
				time.sleep(0.5)
				agent.change_velocity([7,7])
				print("back on track")

			if int(int(data_length/2)) < min_distance_index < 3*int(int(data_length/4)):
				print("stuck in wall")
				agent.change_velocity([-1,-1])
				time.sleep(3)
				agent.change_velocity([1,-1])
				time.sleep(0.5)
				agent.change_velocity([7,7])
				print("back on track")

		# Check if it passed through a door
		if 0 < data_length - data.index(min(data)) < data_length and min(data) < 0.4 and data[data_length - data.index(min(data))] < 0.6 and counter > 1000:
			pass_through_door_flag = True


		#Roughly middle of the room
		if data[int(int(data_length/2))] < 3:
			print("Obstacle Detected. BEEP BEEP!")
			flag_move = False # Stops the robot

	if pass_through_door_flag:
		print("LOOK AT ME, I PASSED THROUGH A DOOR")
		door_counter = 1
		return door_counter
	else:
		door_counter = 0
		return door_counter

	
##########
##########

if __name__ == "__main__":
	plt.ion()
	# Initialize and start the environment
	environment = VrepEnvironment(settings.SCENES + '/room_static.ttt')  # Open the file containing our scene (robot and its environment)
	environment.connect()        # Connect python to the simulator's remote API
	agent   = Pioneer(environment)
	# display = Display(agent, False) 

	print('\nDemonstration of Simultaneous Localization and Mapping using CoppeliaSim robot simulation software. \nPress "CTRL+C" to exit.\n')
	start = time.time()
	step  = 0
	done  = False
	environment.start_simulation()
	time.sleep(1)
	door_counter = 0

	try:    
		while step < settings.simulation_steps and not done:
			# display.update()                      # Update the SLAM display
			door_counter += loop(agent)  
			print(door_counter)                         # Control loop
			step += 1
	except KeyboardInterrupt:
		print('\n\nInterrupted! Time: {}s'.format(time.time()-start))
		
	# display.close()
	environment.stop_simulation()
	environment.disconnect()
