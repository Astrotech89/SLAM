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

def loop(agent):

    """
    Robot control loop
    Your code goes here
    """
    # Example: 
    #agent.change_velocity([1, 0])
    data = agent.read_lidars()

    # Recording the difference in distance between each angle
    differences = []
    for angle in range(269):
        diff = data[angle] - data[angle+1]
        differences.append(diff)

    # Finding the greatest change in distance
    # Index (angle from 0 to 270) where the max is
    for item in differences:
        i = np.where(item > 7.5)
        # If the max is straight ahead
        if i == 135:
            agent.change_velocity([2,2])
        elif i>135:
            agent.change_velocity([2,1])
        elif i<135:
            agent.change_velocity([1,2])

## What needs to get done:
# Don't go through the same door twice
# -- position history
# some width for differences
# only turn right once you're through a door
# What to do when we're in the last room (no doors left)
# How to end up on the red dot

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
