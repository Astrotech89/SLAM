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

def display_lidar(agent, data):
	theta = np.deg2rad(np.arange(0,270))
	# data = agent.read_lidars()
	plt.clf()
	ax = plt.subplot(111,projection='polar')
	ax.set_theta_offset(np.deg2rad(135))
	ax.plot(theta,data)
	plt.show()



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


global door_counter
def loop(agent):
	global door_counter
	count_circle = 980000/2 #amount of counts for a full circle
	
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
	cone_size = 60 # angular size of the cone in degrees
	lower_threshold = 4 # lower threshold for the distance difference between the center and the edge of the cone 
	upper_threshold = 9 # lower threshold for the distance difference between the center and the edge of the cone 
	counter = 1

	data_0 = agent.read_lidars()
	data = data_0[135-cone_size: -135+cone_size]
	data_length = len(data)

	
	print ("Searching for target\nBZZZ")
	agent.change_velocity([0.2,-0.2]) # start rotating

	while flag_rotate: #While condition is true
		

		counter += 1
		data_0 = agent.read_lidars()
		data = data_0[135-cone_size: -135+cone_size]
		data_length = len(data)
		
		# Find the indices of the center (maximum value of the data) and the edges of the cone
		max_distance_index = data.index(max(data))
		middle_index = int(data_length/2)
		upper_boundary_index = middle_index  + cone_size
		lower_boundary_index = middle_index  - cone_size
		
		# print(lower_boundary_index, upper_boundary_index)



		# Find the distance for the center and the edges of the cone
		if lower_boundary_index>=0 and upper_boundary_index <= data_length:
			left_distance = data[lower_boundary_index]
			max_distance = data[max_distance_index]
			right_distance = data[upper_boundary_index-1]
			middle_distance = data[middle_index]


			if lower_threshold < middle_distance-right_distance < upper_threshold:
				if lower_threshold < middle_distance-left_distance < upper_threshold:
					
					print("Target Locked")

					if door_counter == 0:
						time.sleep(1.5)
					if door_counter == 1:
						time.sleep(1)
					if door_counter == 2:
						time.sleep(0.62)
					if door_counter ==3:
						time.sleep(2)

					agent.change_velocity([0,0])
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
	
	
	agent.change_velocity([3,3])
	counter = 1

	# The robot starts moving forward until it sees an obstacle at a distance less than 2 units directly in front of it 
	while flag_move:
		counter += 1
		data_0 = agent.read_lidars()
		data = data_0[135-cone_size: -135+cone_size]
		max_distance_index = data.index(max(data))
		max_distance = data[max_distance_index]

		
			

		# If it gets stuck for too long without an obstacle directly in front of it
		if counter >= 900000:
			print('Nothing to see here, Go Back!')
			reverse(agent)
			flag_move = False # Stops the robot

		# Hit wall condition
		if min(data) < 0.33:
			min_distance_index = data_0.index(min(data_0))
			min_distance = data_0[min_distance_index]

			if int(int(len(data_0)/4)) < min_distance_index < int(int(len(data_0)/2)):
				print("stuck in wall")
				agent.change_velocity([-1,-1])
				time.sleep(3)
				agent.change_velocity([-1,1])
				time.sleep(0.5)
				agent.change_velocity([3,3])
				print("back on track")

			if int(int(len(data_0)/2)) < min_distance_index < 3*int(int(len(data_0)/4)):
				print("stuck in wall")
				agent.change_velocity([-1,-1])
				time.sleep(3)
				agent.change_velocity([1,-1])
				time.sleep(0.5)
				agent.change_velocity([3,3])
				print("back on track")



		if counter >= 900000:
			print('Nothing to see here, Go Back!')
			reverse(agent)
			flag_move = False # Stops the robot



		# Check if it passed through a door
		if 0 < len(data_0) - data_0.index(min(data_0)) < len(data_0) and min(data_0) < 0.4 and data_0[len(data_0) - data_0.index(min(data_0))] < 0.6 and counter > 1000:
			pass_through_door_flag = True


		#Roughly middle of the room
		if data[int(int(data_length/2))] < 3:
			print("Obstacle Detected. BEEP BEEP!")
			flag_move = False # Stops the robot

	if pass_through_door_flag:
		door_counter += 1
		print("LOOK AT ME, I PASSED THROUGH DOOR #", door_counter)
		

	
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
			loop(agent)  
			print(door_counter)                         # Control loop
			step += 1
	except KeyboardInterrupt:
		print('\n\nInterrupted! Time: {}s'.format(time.time()-start))
		
	# display.close()
	environment.stop_simulation()
	environment.disconnect()


