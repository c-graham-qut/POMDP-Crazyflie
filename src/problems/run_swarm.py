# Multi Drone Flight Script
#
# Created by: Marc Schneider (2020)
# Modified by: Conor Graham (2022)
#
# Created for integrating the TAPIR POMDP model with the Bitcraze Crazyflie Drones
# for research at the Queensland University of Technology (QUT) in Brisbane, QLD Australia.
#

import logging
import multiprocessing
import sys
import time
import getch
import math
import cflib.crtp
import numpy as np
import csv

import os
from random import seed
from random import randint
import subprocess
from multiprocessing import Process, Value, Array, Event, Manager, Lock
from datetime import datetime
from datetime import date
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from ctypes import c_char_p

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

# Import Lighthouse Positioning Classes
from cflib.localization.lighthouse_bs_geo import LighthouseBsGeoEstimator
from cflib.localization.lighthouse_bs_vector import LighthouseBsVectors, LighthouseBsVector
from cflib.localization.lighthouse_config_manager import LighthouseConfigFileManager, LighthouseConfigWriter
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.localization import LighthouseBsGeoEstimator
from cflib.localization import LighthouseSweepAngleAverageReader



def wait_for_position_estimator(scf: SyncCrazyflie):
    # Print Function Status
    print('   CONFIGURATION   >>>   Waiting for estimator to find position...')

    # Create Log Configuration Variable
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)

    # Add Variables for the Log Configurations
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    # Initiate varibales for computation
    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    # Set Threshold Level
    threshold = 0.001

    # Open Synclogger
    with SyncLogger(scf, log_config) as logger:
        # Iterate through Log Entry (You only need to iterate through the first entry, this is why a 'break' is added
        # at the bottom of the for loop.
        for log_entry in logger:
            # Retrieve data from log entry
            data = log_entry[1]

            # Add the Kalman Filter variable
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def share_battery_status(scf, Label): #prints battery voltages and and charging status of the drones and changes drone_ready accordingly
  log_config = LogConfig(name='Variance', period_in_ms=10)
  log_config.add_variable('pm.vbat', 'float') # all the log variables have to be added, the name and the data type can be seen in the
  log_config.add_variable('pm.state', 'int8_t') # bitcraze client
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
  wait_for_position_estimator(scf)


def activate_high_level_commander(scf: SyncCrazyflie):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf: SyncCrazyflie, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)


def on_press(key): #waits if left or right ctrl key is pressed and changes break_program or come_home, respectively
    global break_program
    global come_home
    print (key)
    if key == keyboard.Key.ctrl:
        print('   USER INPUT   >>>   CTRL pressed - Session Terminating...')
        emergency_break_sequence.value = 1
        return False
    if key == keyboard.Key.ctrl_r:
        print('coming home')
        emergency_break_sequence.value = 1
        return False


def land_here(scf): #makes drones land by commanding a negative vertical velocity
    cf = scf.cf
    commander = cf.high_level_commander
    if Fly_Go:
        commander.land(0.1, 2.0)
    time.sleep(3)
    commander.stop()
    flight_flag.value = 0
    quit_flag.value = 1
    quit()



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
  #print('\n<<Take Off>>\n')
  commander = cf.high_level_commander
  hover_height = SWARM_hover_height.value
  if Fly_Go:
    commander.takeoff(hover_height, 2.0)
  time.sleep(5)




def set_variables(number_of_drones): #sets all important variables here to avoid having the main program overflow with variable definitions
    # read map.txt
    print('\n   CONFIGURATION   >>>   Setting Global Variables  <<<\n')
    mapfile = open("../../../problems/drones/maps/map.txt", 'r')
    mapsize = mapfile.readline()
    mapfile.close()
    mapsize = mapsize.split(' ')
    map_dimensions = [] # initializes a list
    for numbers in mapsize:
        map_dimensions.append(int(numbers)) # appends map dimensions from file to list

    # position measurements
    rx = [0.]*number_of_drones # [0.] to save it as float instead of int. This way a typecast error can be avoided when adding grid_distance to 0
    ry = [0.]*number_of_drones
    rz = [0.]*number_of_drones
    ryaw = [0.]*number_of_drones
    lh_x = [0.]*number_of_drones
    lh_y = [0.]*number_of_drones
    lh_z = [0.]*number_of_drones

    drone_ready = [1]*number_of_drones
    # initialize a numpy array (np because of "import numpy as np") of zeros
    initial_positions = np.zeros([number_of_drones, 4])
    initial_positions[:,0] = [1]*number_of_drones
    # set x coordinates (i in POMDP, so the vertical position) to the lowest position ( = highest number) possible + 1(map_dimensions[0]-2
    # because C++ starts counting from 0
    # so if map_dimensions[0] was 8 then the lowest position + 1 (i.e. the highest number + 1) is 6.
    initial_positions[:,0] = initial_positions[:,0] * (map_dimensions[0]-2) * SWARM_grid_distance.value
    #set the y coordinates (j in POMDP, so the horizontal position). These positions are the same as in DronesModel::sampleStateUninformed() in
    # the DronesModel.cpp. So if they are changed there they also have to be changed here.
    initial_positions[:,1] = [1]*number_of_drones
    for member in range(number_of_drones):
        if member > (number_of_drones-1)/2:
            #starting position is one grid field from outer column
            initial_positions[member,1] = initial_positions[member,1] * (map_dimensions[1]-1-3) * SWARM_grid_distance.value
        elif member < (number_of_drones-1)/2:
            # if there is an uneven number of drones the middle one starts in the middle of the map
            initial_positions[member,1] = initial_positions[member,1] * (0 + 3) * SWARM_grid_distance.value
        else:
            #starting position is one grid field from outer column
            initial_positions[member,1] = initial_positions[member,1] * (map_dimensions[1]-1)/2 * SWARM_grid_distance.value

    meas_positions = np.zeros([number_of_drones, 4])

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

    return rx, ry, rz, ryaw, drone_ready, initial_positions, meas_positions, map_dimensions, lh_x, lh_y, lh_z


def inMap(position):
    # return true if the position is in map.
    # only positions that are in the map are written to the change file
    global map_dimensions

    # if the position is in the map
    if position[0] <= map_dimensions[0]-1 and position[1] <= map_dimensions[1]-1 and position[0] >= 0 and position[1] >= 0:
        return True
    else:
        return False


def Changes2File(swarm_status, simulation_step, initial_grid_positions):
    position_string = ""  # initialize the string
    gridpositions = np.zeros([number_of_drones, 2])
    for index in range(0, number_of_drones):
        #position_string += "(%d,%d) " % (expected_grid_x[index], expected_grid_y[index])
        gridpositions[index, 0] = expected_grid_x[index]
        gridpositions[index, 1] = expected_grid_y[index]
        snapped_x, snapped_y = snap_to_grid(meas_positions[index, 0], meas_positions[index, 1], initial_grid_positions, index)
        position_string += "(%d,%d) " % (snapped_x, snapped_y)

    position_string += "\n"
    position_file = open("../../../problems/drones/changes/position_measurements.txt", "w")  # append to file
    position_file.write(position_string)
    position_file.close()
    capture_drone_positions(directory_session, swarm_status, meas_positions, simulation_step)
    capture_grid_positions(directory_session, swarm_status, gridpositions, simulation_step)


def round2grid(x, base):
    # returns the number rounded to the next base (used to round the coordinates to the next grid position)
    return np.around(x/base)


def send_swarm_position(scf, position): # make drones go to the position specified for them in 'position' (x,y,z,yaw)
    # Specify Commander Variable
    commander = scf.cf.high_level_commander
    flight_time = SWARM_flight_time.value

    # print('FlyGo = True')
    x = position[0]
    y = position[1]
    z = 0
    yaw = position[3]
    commander.go_to(x, y, z, yaw, flight_time, relative=True)
    time.sleep(flight_time)

    if move_to_position_flag.value == 1:
        SWARM_position_reached.value = SWARM_position_reached.value + 1
        move_to_position_flag.value = 0
    else:
        SWARM_timeout_count.value = SWARM_timeout_count.value + 1

    sending_pos_string = "%.2f,%.2f,%.2f,%.2f\n" % (position[0], position[1], position[2], position[3])
    file_to_open = directory_session + "drone_controller_send_position_log.txt"
    drone_measurement_file = open(file_to_open, "a")  # append to file
    drone_measurement_file .write(sending_pos_string)
    drone_measurement_file .close()


def make_data_folder():
    now = datetime.now()
    # Create Main Data Directory
    main_folder = "../../../problems/drones/" + 'data' + '/' + now.strftime('%d-%b-%Y') + '/'
    # If this directory does not exist - make it
    if not os.path.exists(main_folder):
        os.mkdir(main_folder)
    # Create the target directory
    target_dir = main_folder + now.strftime('%H') + now.strftime('%M') + '/'
    # if the target directory does not exist - make it
    if not os.path.exists(target_dir):
        os.mkdir(target_dir)

    return target_dir


def capture_drone_positions(directory, swarm_status, meas_positions, simulation_step):
    # Open File
    file_name = "measured_drone_positions.csv"
    file_to_open = directory + file_name
    file = open(file_to_open, 'a')
    drone_position_string = ""  # initialize the string

    # Iterate through drone swarm
    for drone in swarm_status:
        execution_time = (time.time() - startTime)
        if drone > 0:
            true_position = meas_positions
            drone_position_string += "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f" % (simulation_step, drone, true_position[drone - 1, 0], true_position[drone - 1, 1], true_position[drone - 1, 2], true_position[drone - 1, 3], execution_time)
            drone_position_string += "\n"
            file.write(drone_position_string)

    # After While loop - close file
    file.close()


def capture_grid_positions(directory, swarm_status, gridpositions, simulation_step):
    # Open File
    file_name = "drone_grid_positions.csv"
    file_to_open = directory + file_name
    file = open(file_to_open, 'a')
    grid_position_string = ""  # initialize the string
    for drone in swarm_status:
        execution_time = (time.time() - startTime)
        if drone > 0:
            grid_position_string += "%d,%d,%d,%d,%.2f " % (simulation_step, drone, gridpositions[drone - 1, 0], gridpositions[drone - 1, 1], execution_time)
            grid_position_string += "\n"
            file.write(grid_position_string)

    # After While loop - close file
    file.close()


def capture_action_states(directory, simulation_step):
    # Get Current Command
    read_file_name = "output.txt"
    read_file_dir = "../../../problems/drones/changes/"
    read_file_path = read_file_dir + read_file_name
    read_file = open(read_file_path, 'r')
    lines = read_file.read().splitlines()
    read_file.close()
    last_line = lines[-1] 
    command = last_line[last_line.find(" ")+1:] 
    
    # Write command to file.
    file_name = "drone_action_states.csv"
    file_to_open = directory + file_name
    file = open(file_to_open, 'a')
    grid_position_string = ""  # initialize the string
    execution_time = (time.time() - startTime)
    grid_position_string += "%d,%d," % (simulation_step, execution_time)
    grid_position_string += command
    grid_position_string += "\n"
    file.write(grid_position_string)
    file.close()


def share_current_states(scf, Label):
    # Define a temporary LogConfig object
    log_config = LogConfig(name='Variance', period_in_ms=10)
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    #log_config.add_variable('stabilizer.yaw', 'float')
    #log_config.add_variable('stabilizer.roll', 'float')
    #log_config.add_variable('stabilizer.pitch', 'float')
    rroll = [0.]*number_of_drones
    rpitch = [0.]*number_of_drones
    # Wrapper SyncLogger will be invoked once and return after one loop
    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            # write the data to the public database
            rx[Label - 1] = data['stateEstimate.x']
            ry[Label - 1] = data['stateEstimate.y']
            rz[Label - 1] = data['stateEstimate.z']
            #ryaw[Label - 1] = data['stabilizer.yaw']
            #rroll[Label - 1] = data['stabilizer.roll']
            #rpitch[Label - 1] = data['stabilizer.pitch']
            meas_positions[Label - 1] = [rx[Label - 1], ry[Label - 1], rz[Label - 1], ryaw[Label - 1]]
            drone_state_estimator_position_string.value = "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n" % (
            timestamp, Label, rx[Label - 1], ry[Label - 1], rz[Label - 1], ryaw[Label - 1], rroll[Label - 1], rpitch[Label - 1])
            file_to_open = directory_session + "estimator_drone_states.txt"
            drone_measurement_file = open(file_to_open, "a")  # append to file
            drone_measurement_file.write(drone_state_estimator_position_string.value)
            drone_measurement_file.close()
            break


def share_current_lh_states(scf, Label):
    # Define a temporary LogConfig object
    log_config = LogConfig(name='Lighthouse_measurements', period_in_ms=10)
    log_config.add_variable('lighthouse.x', 'float')
    log_config.add_variable('lighthouse.y', 'float')
    log_config.add_variable('lighthouse.z', 'float')
    # Wrapper SyncLogger will be invoked once and return after one loop
    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            # write the data to the public database
            lh_x[Label - 1] = data['lighthouse.x']
            lh_y[Label - 1] = data['lighthouse.y']
            lh_z[Label - 1] = data['lighthouse.z']
            drone_state_estimator_position_string.value = "%d,%d,%.2f,%.2f,%.2f\n" % (
            timestamp, Label, lh_x[Label - 1], lh_y[Label - 1], lh_z[Label - 1])
            file_to_open = directory_session + "estimator_lh_drone_states.txt"
            drone_measurement_file = open(file_to_open, "a")  # append to file
            drone_measurement_file.write(drone_state_estimator_position_string.value)
            drone_measurement_file.close()
            break


def limit(input):
    # limits the measurements of the multiranger. According to the spec sheet it has a maximum range of 4 m
    if input > 4000:
        return 4
    else:
        return input/1000 # output by the sensor is in mm, output of the function is in m


def POMDP(): #starts and stops the POMDP solver
    now_time = time.time()
    print('\n   POMDP CONTROLLER   >>>   Start...   <<<   ', now_time - startTime)
    # Initiate While Loop
    while POMDP_stop.value != 1:
        # Wait for Start Command Flag
        if POMDP_start.value == 1:
            print('starting POMDP solver')
            #p = subprocess.call(['gnome-terminal', '-x', './flight'])
            p = subprocess.Popen("./flight", shell=False)
            POMDP_run.value = 1
            while POMDP_run.value == 1:
                # Check if POMDP Solver is still Running
                poll = p.poll()
                if poll != None:
                    # if the subprocess is not running anymore the POMDP must be finished
                    print('\n   POMDP   >>>   Solving process has finished...')
                    POMDP_run.value = 0
                    POMDP_stop.value = 1
                    now_time = time.time()
                    print('\n   POMDP CONTROLLER  >>>   End...   <<<   ', now_time - startTime)
                    p.kill()
                elif emergency_break_sequence.value == 1:
                    print('\n   POMDP   >>>   Emergency Break Sequence has been flagged...')
                    print('\n   POMDP   >>>   Killing POMDP...')
                    POMDP_run.value = 0
                    POMDP_stop.value = 1
                    now_time = time.time()
                    print('\n   POMDP CONTROLLER  >>>   End...   <<<   ', now_time - startTime)
                    p.kill()


def get_unique_values(array):
    unique = []
    for item in array:
        if item in unique:
            continue
        else:
            unique.append(item)

    return unique


# ----------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------- #
# --------------------------------------------ENTRY POINT --------------------------------------------- #
# ----------------------------------------------------------------------------------------------------- #
# ----------------------------------------------------------------------------------------------------- #

# Created by  : Marc Schneider
# Purpose     : The main control mechanism for flight of the drones.
# Modified by : Conor Graham (2022)
def drone_control():
    now_time = time.time()
    print('\n   DRONE CONTROLLER   >>>   Start...   <<<   ', now_time - startTime)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print("\n")
    print('   CONFIGURATION   >>>   Scanning interfaces for Crazyflies...')
    available = []
    count = 0
    while len(available) == 0:
        available = cflib.crtp.scan_interfaces()
        count += 1
        if count > 5:
            print("No Crazyflies found in current session.")
            print("Quitting program...")
            POMDP_stop.value = 1
            quit()
            return

    print("\n")
    print('   CONFIGURATION   >>>   Crazyflies have been found...')
    print("                         ", available)
    uris = []
    uri_dictionary = []
    uri_dict_count = 1
    swarm_dict_count = 1
    swarm_status = []
    drone_number_collection = []
    # Sort Drone Radio Tags
    count = 0
    if len(available) != 0:
        for i in available:
            for j in i:
                if j != '':
                    radio_tag = j
                    drone_number = radio_tag.replace("radio://0/", "")
                    drone_number = drone_number.replace("/", "")
                    drone_number = drone_number.replace("2M", "")
                    drone_number = drone_number.replace(" ", "")
                    drone_number_collection.append(int(str(drone_number)))

        sorted_avaliable = drone_number_collection
        #sorted_avaliable = get_unique_values(sorted_avaliable)
        # Remake the Radio Tags in its original format 'radio://<<Radio Number>>/<<drone number>>/2M'
        for drone in sorted_avaliable:
            count += 1
            if count > 3:
                radio_block_code = 'radio://1/' + str(drone) + '/2M'
            else:
                radio_block_code = 'radio://0/' + str(drone) + '/2M'

            uri_block_code = 'URI' + str(uri_dict_count)
            uris.append(radio_block_code)
            uri_dictionary.append(uri_block_code)
            uri_dict_count += 1
            swarm_status.append(swarm_dict_count)
            swarm_dict_count += 1

    else:
        print('   CONFIGURATION   >>>   No Drones Avaliable...')
        POMDP_stop.value = 1
        quit()
        return

    number_of_drones = len(uris)

    print("\n")
    print('   CONFIGURATION   >>>   Number of Drones for this session...')
    print("                         ", number_of_drones)
    print("\n")
    print('   CONFIGURATION   >>>   Radio Link URIs for this session...')
    print("                         ", uris)
    print("\n")
    print('   CONFIGURATION   >>>   Swarm Status for this session...')
    print("                         ", swarm_status)
    print("\n")

    count = 1
    LABEL = {}
    for item in uris:
        LABEL[item] = [count]
        count += 1

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

        # Activate the Controller
        swarm.parallel(activate_high_level_commander)
        # Reset Estimators

        swarm.parallel(reset_estimator)
        # Start Logging

        # Check drones battery status
        swarm.parallel(share_battery_status, args_dict = LABEL)
        # If drones are not ready quit program
        for i in range(number_of_drones):
            if drone_ready[i] == 0:
                print('drone is still charging! Program exits') # as a safety measure
                # A DRONE THAT IS CHARGING WILL STILL TRY TO FLY IF IT IS COMMANDED TO DO SO
                emergency_break_sequence.value = 1
                quit()
                return

        airborne_flag.value = 0

        print('\n\n   >>>>>>   INITIATING FLYING SEQUENCE  <<<<<<   \n\n')

        swarm.parallel(share_current_states, args_dict=LABEL)

        measurement_flag.value = 1

        x_com_store = np.zeros([number_of_drones, 1])
        y_com_store = np.zeros([number_of_drones, 1])
        z_com_store = np.zeros([number_of_drones, 1])

        init_pos = z_com_store = np.zeros([number_of_drones, 4])

        initial_grid_position_x = np.zeros([number_of_drones, 2])
        initial_grid_position_y = np.zeros([number_of_drones, 2])
        grid_x_store = np.zeros([number_of_drones, 1])
        grid_y_store = np.zeros([number_of_drones, 1])
        print(initial_positions_drones)
        for ii in range(0, number_of_drones):
            initial_grid_position_x[ii, 0] = initial_positions_drones[ii, 0]
            initial_grid_position_x[ii, 1] = meas_positions[ii, 0]
            initial_grid_position_y[ii, 0] = initial_positions_drones[ii, 1]
            initial_grid_position_y[ii, 1] = meas_positions[ii, 1]

        for ii in range(0, number_of_drones):
            x_com_store[ii, 0] = 0
            y_com_store[ii, 0] = 0
            z_com_store[ii, 0] = 0
            grid_x_store[ii, 0] = initial_grid_position_x[ii, 0]
            grid_y_store[ii, 0] = initial_grid_position_y[ii, 0]
            init_pos[ii, 0] = initial_grid_position_x[ii, 0]
            init_pos[ii, 1] = initial_grid_position_y[ii, 0]
            init_pos[ii, 2] = initial_grid_position_x[ii, 1]
            init_pos[ii, 3] = initial_grid_position_y[ii, 1]

        SWARM_takeoff.value = 1

        # RUN-MAIN WHILE LOOP
        # When the program enters this while loop the program is running - everything before this is initiating the drones.
        with keyboard.Listener(on_press=on_press) as listener:

            # Capture the drones initial position
            swarm.parallel(share_current_states, args_dict=LABEL)
            #swarm.parallel(share_current_lh_states, args_dict=LABEL)



            while SWARM_control.value == 0:
                if SWARM_takeoff.value == 1:
                    # Initialise SWARM Command Variables
                    x_com_store, y_com_store, z_com_store, grid_x_store, grid_y_store = new_drone_command(x_com_store,
                                                                                                          y_com_store,
                                                                                                          z_com_store,
                                                                                                          grid_x_store,
                                                                                                          grid_y_store)


                    # Send Command to TAKEOFF
                    now_time = time.time()
                    print('\n   STATUS   >>>   Start Takeoff Sequence...  <<<   ', now_time-startTime, '\n')
                    swarm.parallel(swarm_takeoff)
                    now_time = time.time()
                    print('\n   STATUS   >>>   Takeoff finished  <<<   ', now_time-startTime, '\n')
                    # Start POMDP Process
                    POMDP_start.value = 1

                    # Turn off Take Off Flag
                    SWARM_takeoff.value = 0

                    if emergency_break_sequence.value == 1:
                        # Print SWARM Status
                        print('\n   SWARM   >>>   Emergency Break Sequence has been flagged...')
                        print('\n   SWARM   >>>   Landing Here...')

                        # Send SWARM Land Command
                        swarm.parallel(land_here)

                        # Turn off SWARM Control
                        SWARM_control.value = 0

                        # Stop POMDP Process
                        POMDP_stop.value = 1
                        POMDP_start.value = 1

                        # Stop POMDP from running
                        POMDP_run.value = 0

                        # Quit Process
                        now_time = time.time()
                        print('\n   DRONE CONTROLLER   >>>   End...   <<<   ', now_time - startTime)
                        quit()

                else:
                    # Initialise Loop Variables
                    pos = np.zeros([number_of_drones, 4])

                    # What is the drones next command?
                    x_com_store, y_com_store, z_com_store, grid_x_store, grid_y_store = new_drone_command(x_com_store,
                                                                                                          y_com_store,
                                                                                                          z_com_store,
                                                                                                          grid_x_store,
                                                                                                          grid_y_store)

                    # Convert ccommand to real-reference frame
                    for index in range(0, number_of_drones):
                        pos[index, 0] = x_positions[index]
                        pos[index, 1] = y_positions[index]
                        pos[index, 2] = z_positions[index]

                    # Set Drone Positions
                    com_pos_count = 0
                    POSITIONS = {}
                    for item in uris:
                        content = pos[com_pos_count]
                        POSITIONS[item] = [content]
                        com_pos_count += 1

                    # Send SWARM to that reference frame
                    swarm.parallel(send_swarm_position, args_dict=POSITIONS)

                    # Gather SWARM measurement positions
                    swarm.parallel(share_current_states, args_dict=LABEL)
                    #swarm.parallel(share_current_lh_states, args_dict=LABEL)

                    # Write Change Metrics to Text files
                    if simulation_step.value > 0:
                        Changes2File(swarm_status, simulation_step.value, init_pos)

                    # Check if break conditions have been met
                    if emergency_break_sequence.value == 1:
                        # Print SWARM Status
                        now_time = time.time()
                        print('\n   SWARM   >>>   Emergency Break Sequence has been flagged...')
                        print('\n   SWARM   >>>   Landing Here...   <<<   ', now_time - startTime)

                        # Send SWARM Land Command
                        swarm.parallel(land_here)

                        # Turn off SWARM Control
                        SWARM_control.value = 0

                        # Quit Process
                        now_time = time.time()
                        print('\n   DRONE CONTROLLER   >>>   End...   <<<   ', now_time - startTime)
                        quit()
                    elif POMDP_stop.value == 1:
                        # Print SWARM Status
                        now_time = time.time()
                        print('\n   SWARM   >>>   Landing Here...   <<<   ', now_time - startTime)

                        # Send Land Command
                        swarm.parallel(land_here)

                        # Ensure that this while loop will close
                        SWARM_control.value = 1

                        # Quit
                        now_time = time.time()
                        print('\n   DRONE CONTROLLER   >>>   End...   <<<   ', now_time - startTime)
                        quit()
                    elif SWARM_land.value == 1:
                        # Print SWARM Status
                        now_time = time.time()
                        print('\n   SWARM   >>>   Landing Here...   <<<   ', now_time - startTime)

                        # Send SWARM Land command
                        swarm.parallel(land_here)

                        # Exit POMDP Loop
                        POMDP_stop.value = 1

                        # Exit POMDP Solver Loop
                        POMDP_run.value = 0

                        # Exit SWARM Control Loop
                        SWARM_control.value = 0

                        # Quit this loop process
                        now_time = time.time()
                        print('\n   DRONE CONTROLLER   >>>   End...   <<<   ', now_time - startTime)
                        quit()

                    elif SWARM_timeout_count.value > 10 and POMDP_run.value == 0:
                        # Print SWARM Status
                        print('\n   SWARM   >>>   POMDP has stopped solving without reaching the objective...')
                        print('\n   SWARM   >>>   Landing here...')
                        # Send SWARM Land command
                        swarm.parallel(land_here)

                        # Exit POMDP Loop
                        POMDP_stop.value = 1

                        # Exit POMDP Solver Loop
                        POMDP_run.value = 0

                        # Exit SWARM Control Loop
                        SWARM_control.value = 0

                        # Quit this loop process
                        now_time = time.time()
                        print('\n   DRONE CONTROLLER   >>>   End...   <<<   ', now_time - startTime)
                        quit()


                    # Re-Loop
        listener.join()


def defineObjective():
    filePath = "../../../problems/drones/maps/objective.txt"
    objective = []
    with open(filePath,'r') as f:
        reader = csv.reader(f)
        for row in reader:
          objective.append(float(row[0]))  # Remove the 1st value (x) from the array
          objective.append(float(row[1]))  # Remove the 2nd value (y) from the array
          objective.append(float(row[2]))  # Remove the 3rd value (z) from the array
      
    return objective


def new_drone_command(x_com_store, y_com_store, z_com_store, grid_x_store, grid_y_store):
    x_com = np.zeros([1, number_of_drones])
    y_com = np.zeros([1, number_of_drones])
    z_com = np.zeros([1, number_of_drones])
    grid_x = np.zeros([1, number_of_drones])
    grid_y = np.zeros([1, number_of_drones])
    grid_distance = SWARM_grid_distance.value

    output_file = open("../../../problems/drones/changes/output.txt", "r")
    # reads the file and writes it into a list of strings. Each string is one line in the file
    lines = output_file.read().splitlines()
    output_file.close()
    NewNumberOfLines = len(lines)  # length of the list lines
    last_line = lines[-1]
    # takes the string from the beginning to the space (" ") and casts it into an int
    simulation_step.value = int(last_line[:last_line.find(" ")])

    if NewNumberOfLines != SWARM_command_count.value:  # i.e. a new line (= new command) has been added
        command = last_line[last_line.find(" ") + 1:]  # takes the string after the number (which is the command)
        SWARM_command_count.value += 1
        SWARM_new_command.value = 1
        move_to_position_flag.value = 1

        # the next commands are from the POMDP model, so if they are changed there they also have to be changed here
        if command == "NORTH":  # the drones are in the air but have not received another command yet
            now_time = time.time()
            print('\n   POMDP ACTION   >>>   North   <<<   ', now_time-startTime, '\n')
            x_com = +grid_distance
            y_com = 0
            z_com = 0

            grid_x = -1
            grid_y = 0


        elif command == "SOUTH":  # the drones are in the air but have not received another command yet
            now_time = time.time()
            print('\n   POMDP ACTION   >>>   South   <<<   ', now_time-startTime, '\n')
            x_com = -grid_distance
            y_com = 0
            z_com = 0

            grid_x = 1
            grid_y = 0


        elif command == "EAST":  # the drones are in the air but have not received another command yet
            now_time = time.time()
            print('\n   POMDP ACTION   >>>   East   <<<   ', now_time-startTime, '\n')
            x_com = 0
            y_com = +grid_distance
            z_com = 0

            grid_x = 0
            grid_y = 1


        elif command == "WEST":  # the drones are in the air but have not received another command yet
            now_time = time.time()
            print('\n   POMDP ACTION   >>>   West   <<<   ', now_time-startTime, '\n')
            x_com = 0
            y_com = -grid_distance

            grid_x = 0
            grid_y = -1


        elif command == "NORTH_EAST":  # the drones are in the air but have not received another command yet
            now_time = time.time()
            print('\n   POMDP ACTION   >>>   North East   <<<   ', now_time-startTime, '\n')
            x_com = +grid_distance
            y_com = +grid_distance
            z_com = 0

            grid_x = -1
            grid_y = 1


        elif command == "HOVER":  # move forward by one grid field
            now_time = time.time()
            print('\n   POMDP ACTION   >>>   Hover   <<<   ', now_time-startTime, '\n')
            # No Change to x and y position
            x_com = 0
            y_com = 0
            z_com = 0

            grid_x = 0
            grid_y = 0

        elif command == "LAND":
            now_time = time.time()
            print('\n   POMDP ACTION   >>>   Land   <<<   ', now_time-startTime, '\n')
            x_com = 0
            y_com = 0
            z_com = 0

            grid_x = 0
            grid_y = 0

            SWARM_land.value = 1
        else:
            print('\n\nno action was chosen\n\n')

    if SWARM_new_command.value == 1:
        current_command_value = SWARM_command_count.value
        _x = np.zeros([number_of_drones, 1])
        _y = np.zeros([number_of_drones, 1])
        _z = np.zeros([number_of_drones, 1])

        _gx = np.zeros([number_of_drones, 1])
        _gy = np.zeros([number_of_drones, 1])

        x_com_store = np.append(x_com_store, _x, axis=1)
        y_com_store = np.append(y_com_store, _y, axis=1)
        z_com_store = np.append(z_com_store, _z, axis=1)

        grid_x_store = np.append(grid_x_store, _gx, axis=1)
        grid_y_store = np.append(grid_y_store, _gy, axis=1)

        for ii in range(0, number_of_drones):
            x_com_store[ii][current_command_value] = x_com
            y_com_store[ii][current_command_value] = y_com
            z_com_store[ii][current_command_value] = 0

            grid_x_store[ii][current_command_value] = grid_x_store[ii][current_command_value - 1] + grid_x
            grid_y_store[ii][current_command_value] = grid_y_store[ii][current_command_value - 1] + grid_y

        SWARM_new_command.value = 0

        current_pos_value = SWARM_position_reached.value
        for index in range(0, number_of_drones):
            x_positions[index] = x_com_store[index][current_pos_value]
            y_positions[index] = y_com_store[index][current_pos_value]
            z_positions[index] = 0

            expected_grid_x[index] = grid_x_store[index][current_pos_value]
            expected_grid_y[index] = grid_y_store[index][current_pos_value]
    else:
        current_pos_value = SWARM_position_reached.value
        for index in range(0, number_of_drones):
            x_positions[index] = 0
            y_positions[index] = 0
            z_positions[index] = 0

            expected_grid_x[index] = grid_x_store[index][current_pos_value]
            expected_grid_y[index] = grid_y_store[index][current_pos_value]


    return x_com_store, y_com_store, z_com_store, grid_x_store, grid_y_store


def get_drone_start_pos(number_of_drones):
    mapFile = open("../../../problems/drones/maps/map.txt", "r")  # append to file
    lines = mapFile.readlines()
    lines.pop(0)
    mapFile.close()
    initial_positions = np.zeros([number_of_drones, 2])
    x_count = 0
    y_count = 0
    for line in lines:
        for character in line:
            char = character
            if(char.isdigit()):
                drone_number = int(char)
                initial_positions[drone_number-1] = [x_count, y_count]
            y_count += 1
        y_count = 0
        x_count += 1

    return initial_positions


def snap_to_grid(x_pos, y_pos, initial_grid_positions, drone_number):

    init_pos_x = initial_grid_positions[drone_number, 2]
    init_pos_y = initial_grid_positions[drone_number, 3]
    init_grid_x = initial_grid_positions[drone_number, 0]
    init_grid_y = initial_grid_positions[drone_number, 1]

    if x_pos == 0 and init_pos_x == 0:
        snapped_grid_x = init_grid_x
        snapped_grid_y = init_grid_y + round((y_pos - init_pos_y) / SWARM_grid_distance.value)
    elif y_pos == 0 and init_pos_y == 0:
        snapped_grid_x = init_grid_x - round((x_pos - init_pos_x) / SWARM_grid_distance.value)
        snapped_grid_y = init_grid_y
    elif x_pos == 0 and y_pos == 0 and init_pos_x == 0 and init_pos_y == 0:
        snapped_grid_x = init_grid_x
        snapped_grid_y = init_grid_y
    else:
        snapped_grid_x = init_grid_x - round((x_pos - init_pos_x) / SWARM_grid_distance.value)
        snapped_grid_y = init_grid_y + round((y_pos - init_pos_y) / SWARM_grid_distance.value)

    return snapped_grid_x, snapped_grid_y


#-------------------------------------------------------------------------------------------------------#
# ------------------------------------------------------------------------------------------------------#
#-------------------------------------------- RUN MAIN -------------------------------------------------#
# ------------------------------------------------------------------------------------------------------#
# ------------------------------------------------------------------------------------------------------#

if __name__ == '__main__': #basically handles the two parallel processes which invoke drone_control() and POMDP()


    # safety measure but also a possibility to test if the correct commands are chosen and if all the sensors work.
    # The program runs like normal, only the cf.commander commands are not sent
    print("Queensland University of Technology - QUT")
    print("\n")
    print("Undergraduate Project - Thesis")
    print("2022")
    print("Partially Observable Markov Decision Process")

    print("\n")
    print("   >>>   ", datetime.now(), "   <<<")

    print("\n")
    print('   <<<   START PROGRAM   >>>   ')
    time.sleep(3)

    directory_session = make_data_folder()
    session_objective = defineObjective()

    print("\n")
    print("   CONFIGURATION   >>>   Directory:   ", directory_session)

    Fly_Go = True # WARNING: if Fly_Go is set to True the drone will fly!

    # Fly_Go = False # WARNING: if Fly_Go is set to True the drone will fly!
    capture = False

    # Build C++ program
    subprocess.run(["make", "-j8"])
    subprocess.run(["./solve"])

    # Set Sharable Global Condition Flags:
    take_off_flag = Value('i', 1)
    land_flag = Value('i', 0)
    flight_flag = Value('i', 0)
    simulation_step = Value('i', 0)
    airborne_flag = Value('i', 0)
    command_count = Value('i', 0)

    quit_flag = Value('i', 0)
    measurement_flag = Value('i', 0)
    command_flag = Value('i', 0)
    new_command_flag = Value('i', 0)
    move_to_position_flag = Value('i', 0)

    # Set Controller Flags
    emergency_break_sequence = Value('i',0)

    # Set POMDP Control Flags
    POMDP_stop = Value('i', 0)
    POMDP_start = Value('i', 0)
    POMDP_run = Value('i', 0)

    # Set Swarm Control Flags
    SWARM_control = Value('i', 0)
    SWARM_takeoff = Value('i',0)
    SWARM_land = Value('i', 0)
    SWARM_flight_time = Value('d', 2)
    SWARM_hover_height = Value('d', 0.6)
    SWARM_grid_distance = Value('d', 0.1)
    SWARM_command_count = Value('i', 0)
    SWARM_new_command = Value('i', 0)
    SWARM_position_reached = Value('i', 0)
    SWARM_timeout_count = Value('i', 0)

    # Find Number of Drones in this session:
    number_of_drones = int(input("How many drones for this session?  "))

    # Get Drone Starting Positions
    initial_positions_drones = get_drone_start_pos(number_of_drones)

    # Set the Start Time for program evaluation
    startTime = time.time()

    # Set global variables for this session
    rx, ry, rz, ryaw, drone_ready, initial_positions, meas_positions, map_dimensions, lh_x,lh_y,lh_z = set_variables(number_of_drones)

    # Sets the variables for multithreading
    stop_event = Event()
    manager = Manager()

    # Capture Data Flags
    capture_data_flag = Value('i', 0)

    # Create Shared String
    drone_lighthouse_position_string = manager.Value(c_char_p, "")
    drone_state_estimator_position_string = manager.Value(c_char_p, "")

    # Create Shared Position String
    drone_est_pos = Value('d', 12)
    drone_lh_pos = Value('d', 3)

    # Define Multiprocess Lock Parameter
    lock = Lock()

    # Set Sharable Global Arrays
    x_positions = multiprocessing.Array('d', number_of_drones, lock=lock)
    y_positions = multiprocessing.Array('d', number_of_drones, lock=lock)
    z_positions = multiprocessing.Array('d', number_of_drones, lock=lock)

    expected_grid_x = multiprocessing.Array('d', number_of_drones, lock=lock)
    expected_grid_y = multiprocessing.Array('d', number_of_drones, lock=lock)

    # Define Parallel Process's
    solver_process = Process(target=POMDP)
    flying_process = Process(target=drone_control)

    # Starts the two processes
    solver_process.start(), flying_process.start()
    solver_process.join(), flying_process.join()
    endTime = time.time()
    print('\n   PROGRAM    >>>    Elapsed Time:', endTime -startTime)

    print('\n\n')
    print('   <<<   END PROGRAM   >>>   ')
    print('\n\n')
