"""
"""

from __future__ import print_function
from src.env    import VrepEnvironment
from src.agents import Pioneer
from src.disp   import Display
import settings
import time, argparse
import matplotlib.pyplot as plt

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
			   Angle: [-135:135] Starting with the 
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

	if data[135] == data.max():
		agent.change_velocity([1,1])
		time.sleep(3)
		agent.change_velocity([0,0])
		time.sleep(3)


def reverse(agent):
	'''
	Simply reverses the bot then rotates it
	'''

	agent.change_velocity([-1,-1])
	time.sleep(3)
	agent.change_velocity([-1,1])
	time.sleep(1)
	agent.change_velocity([0,0])

# def pos_hist(agent):
# 	agent.position_history() # Need something with this to help get unstuck
# 	# yeah not sure what this is doing yet, didnt need it until now


# global counter
# counter = 1

	
def loop(agent):
	count_circle = 980000
	
	# numbers that worked:
	# lower_threshold = 2
	# upper_threshold = 9
	# cone_size = 45
	# agent.change_velocity([0.2,-0.2])
	# time.sleep(2.15)

	data = agent.read_lidars()
	flag_rotate = True
	flag_move = False
	lower_threshold = 2
	upper_threshold = 9
	cone_size = 45 
	
	agent.change_velocity([0.2,-0.2])
	counter = 1

	#While condition is true
	while flag_rotate:

		counter += 1
		

		data = agent.read_lidars()
		# pos = agent.position_history
		
		
		# min_distance_index = data.index(min(data))
		max_distance_index = data.index(max(data))
		upper_boundary_index = max_distance_index + cone_size
		lower_boundary_index = max_distance_index - cone_size

		if lower_boundary_index>0 and upper_boundary_index < 270:
			left_distance = data[lower_boundary_index]
			max_distance = data[max_distance_index]
			right_distance = data[upper_boundary_index]
			# print("\n", max_distance - right_distance)
			# print(max_distance - left_distance)
			# print(upper_boundary_index)
			# print(agent.position_history)
			# print(counter)


			
			if max_distance_index > 133 and max_distance_index < 137:
				# if max_distance-right_distance in range(lower_threshold, upper_threshold):
				if lower_threshold < max_distance-right_distance < upper_threshold:
				# if max_distance-right_distance > lower_threshold and max_distance-right_distance < upper_threshold:
					print("nai1")
					# if max_distance-left_distance in range(lower_threshold, upper_threshold):
					if lower_threshold < max_distance-left_distance < upper_threshold:
					# if max_distance-left_distance > lower_threshold and max_distance-left_distance < upper_threshold:
					
						time.sleep(2.15)
						agent.change_velocity([0,0])
						print(data.index(data[lower_boundary_index]))
						print(data.index(data[max_distance_index]))
						print(data.index(data[upper_boundary_index]))
						
						flag_rotate = False
						flag_move = True

		#If robot has completed a full circle without moving.  Counts determined by experimentation
		if counter >= count_circle * 1:
			print('No Detection')
			got_stuck_spin(agent)
			counter = 1 #Resets counter
			flag_move = False #Resets loop
			flag_rotate = False #Same
	# print(counter)
	
	
	agent.change_velocity([7,7])
	counter = 1
	# middle_step = 0
	#While tuple is true
	while flag_move:
		counter += 1
		data = agent.read_lidars()
		max_distance_index = data.index(max(data))
		max_distance = data[max_distance_index]

		# pos = 
		#Hit wall condition
		if data[135] < 0.35:
			print('Hit Wall')
			got_stuck_spin(agent)
			flag_move = False
			

		#Run on condition aka it's hit something and isn't moving
		if counter >= 300000:
			print('Run on condition')
			reverse(agent)
			flag_move = False
		#Roughly middle of the room
		if data[135] < 2:
			print("eftasa")
			flag_move = False

	
##########
##########

if __name__ == "__main__":
	plt.ion()
	# Initialize and start the environment
	environment = VrepEnvironment(settings.SCENES + '/room_static.ttt')  # Open the file containing our scene (robot and its environment)
	environment.connect()        # Connect python to the simulator's remote API
	agent   = Pioneer(environment)
	display = Display(agent, False) 

	print('\nDemonstration of Simultaneous Localization and Mapping using CoppeliaSim robot simulation software. \nPress "CTRL+C" to exit.\n')
	start = time.time()
	step  = 0
	done  = False
	environment.start_simulation()
	time.sleep(1)

	try:    
		while step < settings.simulation_steps and not done:
			display.update()                      # Update the SLAM display
			loop(agent)                           # Control loop
			step += 1
	except KeyboardInterrupt:
		print('\n\nInterrupted! Time: {}s'.format(time.time()-start))
		
	display.close()
	environment.stop_simulation()
	environment.disconnect()
