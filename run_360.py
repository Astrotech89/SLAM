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
		  3. agent.read_lidars()   
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

###########
###########
import random

def check_stuck(agent):
	pos = agent.position_history()
	return pos[0:] == pos[:-1]

def random(minpos):
	if minpos<135:
		agent.change_velocity([1,-1])	
		time.sleep(random.randint(1,9)) 
	else:
		agent.change_velocity([-1,1])	
		time.sleep(random.randint(1,9)) 
	agent.change_velocity([3,3])
	time.sleep(4)          

global j
j=0

def loop(agent):
	global j
	
	#read the lidars, find closest object	
	data = agent.read_lidars()
	minpos = data.index(min(data))

	#turn to face closest object if it isn't on your right
	if minpos<40 or minpos>50:     
		while minpos!=135:
			agent.change_velocity([1,-1])
			time.sleep(0.5)
			data = agent.read_lidars()
			minpos = data.index(min(data))
		#go towards closest object
		while data[135]>=0.7:
			agent.change_velocity([3,3])

	#turn to have object on your side
	while minpos!=45:
		agent.change_velocity([-1,1]) 
		data = agent.read_lidars()
		minpos = data.index(min(data)) 
	
	#follow wall
	agent.change_velocity([3,3])
	time.sleep(2)

	#free yourself if you get stuck  
	if check_stuck(agent):
		agent.change_velocity([-2,-2])	
		time.sleep(1) 
		agent.change_velocity([1,-1])
		time.sleep(1)  

	#do a random move to not get stuck around an "island"
	j=j+1
	if j == 10000:
		random(minpos)
		j=0


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
