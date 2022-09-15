# Multi Drone Flight Script
#
# Created by: Marc Schneider (2020)
# Modified by: Conor Graham (2022)
#
# Created for integrating the TAPIR POMDP model with the Bitcraze Crazyflie Drones
# for research at the Queensland University of Technology (QUT) in Brisbane, QLD Australia.
#

import logging
import sys
import time
import getch
import math
import cflib.crtp
import numpy as np

import os
from random import seed
from random import randint
import subprocess
from multiprocessing import Process, Value, Array, Event

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from pynput import keyboard
from cflib.crazyflie import Crazyflie
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

def share_battery_status(scf, Label): #prints battery voltages and and charging status of the drones and changes drone_ready accordingly
  log_config = LogConfig(name='Variance', period_in_ms=10)
  log_config.add_variable('pm.vbat', 'float') # all the log variables have to be added, the name and the data type can be seen in the
  log_config.add_variable('pm.state', 'int8_t') # bitcraze client
  print('\n')
  with SyncLogger(scf, log_config) as logger:
    for log_entry in logger:
      # print('logger = ', logger)
      data = log_entry[1]
      # drone is set ready. Because this function is used by the swarm.parallel() function each drone will get its own label.
      # (drone 1 will get Label 1, drone 2 will get Label 2, ...)
      drone_ready[Label-1] = 1 # Label-1 because Python starts counting from 0
      if data['pm.state'] != 0 and Fly_Go == True:
        drone_ready[Label-1] = 0 # as a safety measure (even if Fly_Go == True but the drone is still attached to the charging cable it will
                                 # be labelled as not ready)
      if data['pm.state'] == 0:
        status = 'not charging'
      elif data['pm.state'] == 1:
        status = 'charging finished'
      elif data['pm.state'] == 2:
        status = 'charging'
      else:
        status = 'unknown battery status'
        # anything above 3.8 V is OK, lower voltage causes the drone to wobble around and sometimes fly unstable (the lower the voltage the bigger
        # the effect)
      print('drone = ', Label, 'voltage = ', data['pm.vbat'], 'status = ', status) 
      return

def reset_estimator(scf): 
  #resets current position estimate of drones to be (0,0,0,0 [x,y,z,yaw]), otherwise the Flowdeck will output a random position
  # IMPORTANT: If you use the Loco Positioning System (LPS) the position MUST NOT be reset
  cf = scf.cf
  cf.param.set_value('kalman.resetEstimation', '1')
  time.sleep(0.1)
  cf.param.set_value('kalman.resetEstimation', '0')
  time.sleep(2)

def on_press(key): #waits if left or right ctrl key is pressed and changes break_program or come_home, respectively
    global break_program
    global come_home
    print (key)
    if key == keyboard.Key.ctrl:
        print('end pressed')
        break_program = True
        return False
    if key == keyboard.Key.ctrl_r:
        print('coming home')
        come_home = True
        return False

def land_here(scf): #makes drones land by commanding a negative vertical velocity
    cf = scf.cf
    print('landing here...\n')
    for t in range(1): # 1 loops with 1 second each (commands ALWAYS have to be in a loop, even if the loop has only one iteration)
        if Fly_Go: 
            # commands can be looked up in the commander.py file
            cf.commander.send_velocity_world_setpoint(0, 0, -0.4, 0) # sets the velocity of the drone in world coordinates (vx,vy,vz,yawrate)[m/s]
        # lands
        time.sleep(0.1)            
    print('Demo terminated!\n')

def land_at_home(scf): # makes drone fly to home (= 0,0,0,0 [x,y,z,yaw]) position then invokes land_here() function
    cf = scf.cf
    print('landing at home...')
    for t in range(30): # 3 loops with 1 second each
        if Fly_Go:
            cf.commander.send_position_setpoint(0, 0, 0.4, 0) # makes the drone go to this position (x,y,z,yaw)
        # lands
        time.sleep(0.1)            
    for t in range(5):
        if Fly_Go:
            cf.commander.send_velocity_world_setpoint(0, 0, -0.3, 0)
        # lands
        time.sleep(0.1)            
    print('Demo home terminated!\n')

def swarm_takeoff(scf): #makes drones take off (first with velocity commander, then with the position commander)
  cf = scf.cf
  # print('swarm is taking off', hover_height)
  for i in range(10):
    if Fly_Go:
    # make the propellers spin and make the drone hover a bit in ground effect. This allows for a smoother takeoff because the position 
    # controller (send_position_setpoint) is very agressive and overshoots when starting from the ground
      cf.commander.send_velocity_world_setpoint(0, 0, i/10, 0) 
    time.sleep(0.05)
  for i in range(10):
    if Fly_Go:
      cf.commander.send_position_setpoint(0, 0, hover_height, 0)
    time.sleep(0.1)
  print('\ntakeoff finished\n')

def set_variables(): #sets all important variables here to avoid having the main program overflow with variable definitions
    # read map.txt
    mapfile = open("../../../problems/drones/maps/map.txt", 'r')
    mapsize = mapfile.readline()
    mapfile.close()
    mapsize = mapsize.split(' ')
    map_dimensions = [] # initializes a list
    for numbers in mapsize:
      map_dimensions.append(int(numbers)) # appends map dimensions from file to list
    break_program = False
    come_home = False
    hover_height = 0.6
    MIN_DISTANCE = 0.4
    MAX_HEIGHT = 1.5
    grid_distance = 0.2 # 0.2
    big_delta = 0.4
    critical_dist = 0.2
    wait_time = 0.15
    NrOfDrones = 5 # this is the maximum number of drones. It is possible to have it set to 5 but only fly with less drones
    NrOfObstacles = 3
    # position measurements
    rx = [0.]*NrOfDrones # [0.] to save it as float instead of int. This way a typecast error can be avoided when adding grid_distance to 0
    ry = [0.]*NrOfDrones
    rz = [0.]*NrOfDrones
    ryaw = [0.]*NrOfDrones

    drone_ready = [1]*NrOfDrones
    # initialize a numpy array (np because of "import numpy as np") of zeros
    initial_positions = np.zeros([NrOfDrones, 4]) 
    initial_positions[:,0] = [1]*NrOfDrones
    # set x coordinates (i in POMDP, so the vertical position) to the lowest position ( = highest number) possible + 1(map_dimensions[0]-2
    # because C++ starts counting from 0
    # so if map_dimensions[0] was 8 then the lowest position + 1 (i.e. the highest number + 1) is 6.
    initial_positions[:,0] = initial_positions[:,0] * (map_dimensions[0]-2) * grid_distance
    #set the y coordinates (j in POMDP, so the horizontal position). These positions are the same as in DronesModel::sampleStateUninformed() in
    # the DronesModel.cpp. So if they are changed there they also have to be changed here. 
    initial_positions[:,1] = [1]*NrOfDrones 
    for member in range(NrOfDrones):
      if member > (NrOfDrones-1)/2:
        #starting position is one grid field from outer column
        initial_positions[member,1] = initial_positions[member,1] * (map_dimensions[1]-1-3) * grid_distance
      elif member < (NrOfDrones-1)/2:
        # if there is an uneven number of drones the middle one starts in the middle of the map
        initial_positions[member,1] = initial_positions[member,1] * (0 + 3) * grid_distance
      else:
        #starting position is one grid field from outer column
        initial_positions[member,1] = initial_positions[member,1] * (map_dimensions[1]-1)/2 * grid_distance
    meas_positions = np.zeros([NrOfDrones, 4])
    # x y z yaw

    # print('initial_positions = ')
    # print(initial_positions)
    firstfile = open("../../../problems/drones/changes/output.txt", 'w')
    # overwrites the file with 0 HOVER. This is needed to erase the old commands and make the drones takeoff as a first step (step 0)
    firstfile.write("0 HOVER\n")
    firstfile.close()
    # overwrites changes file with nothing (= erases the file content)
    input_file = open("../../../problems/drones/changes/collision-changes.txt", "w")
    input_file.close()
    # overwrites position_file with nothing (= erases the file content)
    position_file = open("../../../problems/drones/changes/position_measurements.txt", "w")
    position_file.close()

    return break_program, come_home, hover_height, MIN_DISTANCE, MAX_HEIGHT, grid_distance, big_delta, critical_dist, wait_time, \
    rx, ry, rz, ryaw, drone_ready, NrOfDrones, initial_positions, meas_positions, front, left, right, back, up, map_dimensions

def trajectoryCommand():
    output_file = open("../../../problems/drones/changes/output.txt","r")
    lines = output_file.read().splitlines()
    output_file.closed()

    if airborne == FALSE:
        print('TAKEOFF')
        takeoff = True
    elif command == "TRAVERSE":
        print('\n\nAction TRAVERSE chosen\n\n')
    elif command == "AVOID":
        print('\n\nAction AVOID chosen\n\n')
    elif command == "HOVER":
        x_com =
        y_com =
    elif command == "LAND":
        print('\n\nAction LAND chosen\n\n')
        command_land = True


    return pos, NewNumberOfLines, takeoff, command_land



def calc_command(hover_height, grid_distance, x_com, y_com, NumberOfLines, airborne, swarm_status, meas_positions, obstacles): #calculates next position for the drones depending on POMDP output
  output_file = open("../../../problems/drones/changes/output.txt", "r")
  # reads the file and writes it into a list of strings. Each string is one line in the file
  lines = output_file.read().splitlines()
  output_file.close()
  NewNumberOfLines = len(lines) #length of the list lines
  takeoff = False
  command_land = False
  # [-1] takes the last entry of the list
  last_line = lines[-1] 
  # takes the string from the beginning to the space (" ") and casts it into an int
  simulation_step = int(last_line[:last_line.find(" ")])
  real_positions = initial_positions - meas_positions
  gridpositions = round2grid(real_positions, grid_distance) # x, y, z, yaw
  if simulation_step > 0:
    Changes2File(gridpositions, grid_distance, swarm_status, simulation_step, obstacles)
  if NewNumberOfLines != NumberOfLines: # i.e. a new line (= new command) has been added

    print(' \n\ngridpositions = ')
    print(gridpositions)

    command = last_line[last_line.find(" ")+1:] # takes the string after the number (which is the command)
    if airborne == False: # if the drones are not in the air yet the first thing they do is take off
      print('TAKEOFF')
      takeoff = True

    # the next commands are from the POMDP model, so if they are changed there they also have to be changed here
    elif command == "HOVER": # the drones are in the air but have not received another command yet
      print('\n\nAction HOVER chosen\n\n')
      #x_com and y_com are not changed
    elif command == "FORWARD": # move forward by one grid field 
      print('\n\nAction FORWARD chosen\n\n')
      x_com += grid_distance
    elif command == "WIDER":
      #positive in y-direction means it goes left (from the drones initial point of view)
      y_com += [grid_distance, grid_distance, 0, - grid_distance, - grid_distance] # if the NumberOfDrones changes this vector has to be changed as well
      print('\n\nAction WIDER chosen\n\n')
    elif command == "NARROWER":
      #positive in y-direction means it goes left (from the drones initial point of view)
      y_com -= [grid_distance, grid_distance, 0, - grid_distance, - grid_distance] # if the NumberOfDrones changes this vector has to be changed as wells
      print('\n\nAction NARROWER chosen\n\n')
    elif command == "REARRANGE":
      print('\n\nAction REARRANGE chosen\n\n')
    elif command == "LAND":
      print('\n\nAction LAND chosen\n\n')
      command_land = True
    else:
      print('\n\nno action was chosen\n\n')
    
  # constructs the commanded positions for the drones
  pos = np.zeros([NrOfDrones, 4]) # 4 because it contains x,y,z,yaw
  pos[:,0] = x_com
  pos[:,1] = y_com
  pos[:,2] = hover_height


  return pos, NewNumberOfLines, takeoff, command_land

def inMap(position):
  # return true if the position is in map.
  # only positions that are in the map are written to the change file
  global map_dimensions

  # if the position is in the map
  if position[0] <= map_dimensions[0]-1 and position[1] <= map_dimensions[1]-1 and position[0] >= 0 and position[1] >= 0:
    return True
  else:
    return False

# Created By: Conor Graham
# Purpose:
def CollisionCheck(gridpositions, meas_positions, grid_distance, swarm_status, simulation_step, obstacles, tolerance):
    # Obstacles - ARRAY - M x N - (TotalObstacleCount x 3)                    - need to be in an [x; y; z] format (vertical vector/array) or (MxN = 3x1)
    # dist      - ARRAY - M x N - (TotalObstacleCount x TotalDronesInSession) - this array will hold float values of the distance between the obstacle and the drone.
    # collsion  - BOOL  - M x N - (TotalObstacleCount x TotalDronesInSession) - if a drone is within a specific distance away from an obstacle it is TRUE.

    collision = np.zzeros([NrOfDrones,NrOfObstacles], dtype=float)
    drone_pos = np.zzeros([NrOfDrones,3], dtype=float)
    dist = np.zeros([NrOfDrones,1], dtype=float)
    drone_pos[:,0] = [meas_positions[:,0]]
    drone_pos[:,1] = [meas_positions[:,1]]
    drone_pos[:,2] = [meas_positions[:,2]]

    for drone in drone_pos:
        for obstacle in obstacles:
            dist = math.dist(drone_pos(drone-1),:), obstacle)
            if dist < tolerance:
                collision[obstacle,drone-1] = 1
            else:
                collision[obstacle, drone-1] = 0

    # Check if collision array holds a positive marker (i.e 1)
    if collision == 1:
        input_file = open("../../../problems/drones/changes/collision-changes.txt", "a")  # append to file
        input_file.write(first_string)
        input_file.write(second_string)
        input_file.close()

    return collision

def Changes2File(gridpositions, meas_positions, grid_distance, swarm_status, simulation_step, obstacles):
  # open multiranger-changes.txt file and write 'X' at the position of an obstacle;
  # As this function is invoked directly before the commands are sent to the drones it can be assumed that the drones reached their
  # steady state positions (which should be the same as in the POMDP gridspace). From here obstacles can be detected and the changes
  # can be written into the multiranger-changes.txt file.

  for drone in swarm_status:
      if drone > 0 and meas_positions[drone-1,0]

  i = 0 # number of changes that are going to be written into the file
  dist = 7 # maximum number of grid fields to detect obstacles (obstacles further away will not be detected)
  first_string = "" # initialize the string
  second_string = "" # initialize the string
  position_string = "" # initialize the string
  for drone in swarm_status:
    
    # obstacle in front of drone (multiranger_meas[drone - 1, 0] = multiranger.front)
    if drone > 0 and meas_positions[drone - 1, 0] < dist * grid_distance and \
     inMap([gridpositions[drone-1,0]-round2grid(meas_positions[drone - 1, 0], grid_distance) + 1, gridpositions[drone-1,1]]):
        i += 1
        # Order of the coordinates is South, then East, first pair is one corner of a rectangle, second pair is the opposite corner 
        # of the rectangle of changes.
        # The first entry of the pair MUST be the top-left corner, the other one the bottom-right one
        # obstacle in front
        second_string += "Add Obstacles: (%d, %d) (%d, %d)\n" \
        % (gridpositions[drone-1,0]-round2grid(meas_positions[drone - 1, 0], grid_distance) + 1, gridpositions[drone-1,1], \
         gridpositions[drone-1,0]-round2grid(meas_positions[drone - 1, 0], grid_distance) + 1, gridpositions[drone-1,1])

    if drone > 0:
      position_string += "(%d,%d) " % (gridpositions[drone-1,0], gridpositions[drone-1,1])

  first_string = "t %d : %d \n" %(simulation_step+1, i)

  position_string += "\n"

  if i > 0: # only wirte to file if there are changes
    input_file = open("../../../problems/drones/changes/multiranger-changes.txt", "a") # append to file
    input_file.write(first_string)
    input_file.write(second_string)
    input_file.close()

  position_file = open("../../../problems/drones/changes/position_measurements.txt", "w") # append to file
  position_file.write(position_string)
  position_file.close()



def round2grid(x, base):
    # returns the number rounded to the next base (used to round the coordinates to the next grid position)
    return np.around(x/base)

def send_swarm_position(scf, position): # make drones go to the position specified for them in 'position' (x,y,z,yaw)
  cf = scf.cf
  # print('sending positions: ', position[0], position[1], position[2], position[3])
  if Fly_Go:
    # print('FlyGo = True')
    cf.commander.send_position_setpoint(position[0], position[1], position[2], position[3])

def share_current_states(scf, Label): #make drones write their current x, y, z and yaw measurements into global variables
  # Define a temporary LogConfig object
  log_config = LogConfig(name='Variance', period_in_ms=10)
  log_config.add_variable('stateEstimate.x', 'float')
  log_config.add_variable('stateEstimate.y', 'float')
  log_config.add_variable('stateEstimate.z', 'float')
  log_config.add_variable('stabilizer.yaw', 'float')
  # Wrapper SyncLogger will be invoked once and return after one loop
  with SyncLogger(scf, log_config) as logger:
    for log_entry in logger:
      data = log_entry[1]
      # write the data to the public database
      rx[Label-1] = data['stateEstimate.x']
      ry[Label-1] = data['stateEstimate.y']
      rz[Label-1] = data['stateEstimate.z']
      ryaw[Label-1] = data['stabilizer.yaw']
      meas_positions[Label-1] = [rx[Label-1],ry[Label-1],rz[Label-1],ryaw[Label-1]]
      return

def limit(input):
  # limits the measurements of the multiranger. According to the spec sheet it has a maximum range of 4 m
  if input > 4000:
    return 4
  else:
    return input/1000 # output by the sensor is in mm, output of the function is in m

# Created by  : Marc Schneider
# Purpose     : Run the POMDP solver.
# Modified by :
def POMDP(n, a, stop_event): #starts and stops the POMDP solver
  # a.value == 0 when program starts, 1 when POMDP finishes and 2 when drones are charging or not available
  while a.value != 1 and not stop_event.is_set():
    while n.value == 0: #n.value == 0 when drones are not in the air, n.value == 1 when drones are in the air
      if a.value == 2: 
        break # break out the loop if the drones are charging or not available
      time.sleep(0.1) # wait to start POMDP solver until drone is ready

    if n.value == 1:
      print('starting POMDP solver')
      p = subprocess.Popen("./flight", shell=False)
      while a.value == 0 and not stop_event.is_set():
        # finds out if subprocess p is still running
        poll = p.poll()
        if poll != None:
          # if the subprocess is not running anymore the POMDP must be finished
          print('\n\n\n\n\nPOMDP finished \n\n\n\n\n')
          a.value = 1 
          n.value = 2 # to prevent the while loop in the beginning of the funtion
        time.sleep(1)
  if stop_event.is_set():
    # kills the POMDP process otherwise it spams the console until it is finished
    p.kill()

# Created by  : Conor Graham
# Purpose     : Initialise the connection of the drones for the session.
def initiate_drones():
    cflib.crtp.init_drivers(enable_debug_drivers=False)
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    organized_available = []
    for i in available:
        print(i[0])
        # prints the adresses of the available drones
        organized_available.append(i[0][9:-1])
        # appends the adress to the list

    print('\n')

    swarm_status = [0] * NrOfDrones
    uris = []

    if len(sys.argv) == 1:
        # checks the length of the input arguments. The program itself is the first input argument
        # (e.g. "python3 multidrone_flight_test.py")

        if len(available) < 3:
            # less than 3 drones --> one antenna is enough
            URI1 = 'radio://0/60/2M'
            URI2 = 'radio://0/70/2M'
            URI3 = 'radio://0/80/2M'
            URI4 = 'radio://0/90/2M'
            URI5 = 'radio://0/100/2M'

        else:
            # at least 3 drones --> use two antennas
            URI1 = 'radio://0/60/2M'
            URI2 = 'radio://0/70/2M'
            URI3 = 'radio://0/80/2M'
            URI4 = 'radio://1/90/2M'
            URI5 = 'radio://1/100/2M'

        for i in available:
            if i[0][9:-1] == URI1[9:-1]:
                uris.append(URI1)
                swarm_status[0] = 1
            elif i[0][9:-1] == URI2[9:-1]:
                uris.append(URI2)
                swarm_status[1] = 2
            elif i[0][9:-1] == URI3[9:-1]:
                uris.append(URI3)
                swarm_status[2] = 3
            elif i[0][9:-1] == URI4[9:-1]:
                uris.append(URI4)
                swarm_status[3] = 4
            elif i[0][9:-1] == URI5[9:-1]:
                uris.append(URI5)
                swarm_status[4] = 5
        if len(uris) == 0:
            print('no crazyflie found, please cancel the program and try again. If it still does not work try turning the crazyflie off and on  \
          or plug the antenna out and in again')
            return

    print('swarm_status = ', swarm_status)

    # creates the LABEL. This set will be used when sending commands to multiple drones (with swarm.parallel) to tell each drone which number is
    # assigned to it. (URI1 = 'radio://0/20/2M' will be assigned to drone 1, ...)
    LABEL = {
        URI1: [1],
        URI2: [2],
        URI3: [3],
        URI4: [4],
        URI5: [5]
            }
    return LABEL, swarm_status, uris

# ----------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------- #
# --------------------------------------------ENTRY POINT --------------------------------------------- #
# ----------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------- #

# Created by  : Marc Schneider
# Purpose     : The main control mechanism for flight of the drones.
# Modified by : Conor Graham (2022)
def drone_control(n, a, stop_event):
    # Initiate the drones
    LABEL, swarm_status, uris = initiate_drones():

    # RUN-MAIN WHILE LOOP
    # When the program enters this while loop the program is running - everything before this is initiating the drones.
    with keyboard.Listener(on_press=on_press) as listener:
      # the keyboard.Listener is needed to make the drones land immediately if the one of the ctrl keys is pressed.
      # left ctrl: land_here()
      # right ctrl: land_at_home()

      #a.value == 1 when POMDP is finished, a.value == 2 to stop program (unavailable drone chosen or drone charging)
      while a.value == 0 and break_program == False and come_home == False:
        # saves position measurements in global variables x, y, z, yaw
        swarm.parallel(share_current_states, args_dict = LABEL)
        # calculates the commands for the drones
        com_positions, NumberOfLines, takeoff, land_command = calc_command(hover_height, grid_distance, x_com, y_com, NumberOfLines, airborne, swarm_status, meas_positions, obstacles)
        x_com = com_positions[:,0]
        y_com = com_positions[:,1]

        # Set Drone Positions
        POSITIONS = {
          URI1: [com_positions[0]],
          URI2: [com_positions[1]],
          URI3: [com_positions[2]],
          URI4: [com_positions[3]],
          URI5: [com_positions[4]]
          }

        if takeoff == False:
          # sends to the commanded positions to the drones. Here the input argument for the drones is POSITIONS which connects the URI to the
          # commanded position for that drones.
          # To send commands with an input argument to multiple drones a set with the URIs and then the argument is needed so the correct argument
          # can be sent to the correct adress (= drone)
          swarm.parallel(send_swarm_position, args_dict = POSITIONS)
        else:
          n.value = 1
          # the command swarm_takeoff does not have any input arguments, so no set containing the URIs is needed
          swarm.parallel(swarm_takeoff)
          # if the swarm took off airborne is set to True
          airborne = True
        if land_command == True:
          swarm.parallel(land_here)
          break
        time.sleep(wait_time)
        # print('a.value = ', a.value)
      # end of while loop

      if break_program == True:
        # left ctrl key was pressed
        print('\n\n\nkilling POMDP process\n\n\n')
        stop_event.set()
        swarm.parallel(land_here)
      elif come_home == True:
        # right ctrl key was pressed
        print('\n\n\nkilling POMDP process\n\n\n')
        stop_event.set()
        swarm.parallel(land_at_home)
      else:
        # POMDP solver finished. Then the drones land at their current position
        stop_event.set()
        swarm.parallel(land_here)
    listener.join()


#-------------------------------------------------------------------------------------------------------#
# ------------------------------------------------------------------------------------------------------#
#-------------------------------------------- RUN MAIN -------------------------------------------------#
# ------------------------------------------------------------------------------------------------------#
# ------------------------------------------------------------------------------------------------------#

if __name__ == '__main__': #basically handles the two parallel processes which invoke drone_control() and POMDP()
    
    # safety measure but also a possibility to test if the correct commands are chosen and if all the sensors work.
    # The program runs like normal, only the cf.commander commands are not sent

    Fly_Go = True # WARNING: if Fly_Go is set to True the drone will fly!
    # Fly_Go = False # WARNING: if Fly_Go is set to True the drone will fly!
    
    # Build C++ program
    subprocess.run(["make", "-j8"])
    subprocess.run(["sudo","./solve"])

    # Sets all the important variables
    break_program, come_home, hover_height, MIN_DISTANCE, MAX_HEIGHT, grid_distance, big_delta, critical_dist, wait_time, \
    rx, ry, rz, ryaw, drone_ready, NrOfDrones, initial_positions, meas_positions, multiranger_meas, front, left, right, back, up, map_dimensions = set_variables()
    # Sets the variables for multithreading
    stop_event = Event()
    n = Value('i', 0)
    a = Value('i', 0)
    solver_process = Process(target=POMDP, args=(n, a, stop_event))
    flying_process = Process(target=drone_control, args=(n, a, stop_event))
    # Starts the two processes
    solver_process.start(), flying_process.start()
    solver_process.join(), flying_process.join()
    print('python script finished')
