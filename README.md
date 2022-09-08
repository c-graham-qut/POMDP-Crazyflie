# Implementation of POMDP using Bitcraze Crazyflie Drones
This repository is set up to develop the necessary software to implement a POMDP model using the Bitcraze Crazyflie Drones for a thesis topic at the Queensland Univeristy of Technology (QUT). 

## Requirements
### Software
1. Operating System -> Ubuntu v20.04

### Hardware
1. Laptop or Desktop
2. [Bitcraze Crazyflie Drones v2.1](https://store.bitcraze.io/products/crazyflie-2-1)
4. [Bitcraze CrazyRadio Dongle](https://store.bitcraze.io/collections/accessories/products/crazyradio-pa)
5. [Infrared Positioning Systems](https://store.bitcraze.io/collections/positioning/products/lighthouse-v2-base-station)

## Overview
This toolkit is being used as it provides a framework that can be integrated with the Bitcraze Crazyflie drones. 

For more information on the POMDP model used please visit -> [TAPIR POMDP - Master](https://github.com/RDLLab/tapir)

## Toolkit Implementation

### Installation
The following procedure will demonstrate how to install this software toolkit.

#### Step 1 - Download the entire repository into a folder of your choise.
#### Step 2 - Open a new terminal window.
#### Step 3 - Navigate to the repository location.
#### Step 4 - Initiate the run file:
The 'run.sh' file downloads the necessary repositories for the POMDP TAPIR toolkit and the Bitcraze Crazyflies. The file includes the following commands: 

Run the following code in the open terminal:
```
./run
```

##### Run File Contents
The code below downloads the repositories for the Bitcraze Crazyflie Client
```
sudo add-apt-repository ppa:ubunutu-toolchain-r/test
sudo apt-get update
```
The code below downloads and installs python, pip, pynput, keyboard and numpy repositories.
```
sudo apt-get install python3
sudo apt install git python3-pip libxcb-xinerama0
sudo pip3 install --upgrade pip
sudo pip3 install pynput
sudo pip3 install keyboard
sudo pip3 install numpy
```
The code below downloads and installs the Crazyflie Library
```
sudo pip3 install cflib
```
The code below distributes specific binary, header and C++ files for the POMDP TAPIR Toolkit
```
cp -avr $(pwd)/Spatial-Index-Components/include/spatialindex /usr/include
rsync -a $(pwd)/Spatial-Index-Components/lib/ /usr/lib
```
The code below initiates the 'make' file and starts to generate the necessary binary files for the program to run appropiately:
```
sudo make all
```
The code below installs the Crazyflie Client
```
sudo git clone https://github.com/bitcraze/crazyflie-clients-python
sudo cd crazyflie-clients-python
sudo pip3 install -e .
```
The code below sets XAuthority for all users on the device in order to connect to the display (:0).
```
sudo cd
xhost +si:localuser:root
```




## General Information

### Communication 
The Crazyflie radio dongles operate within the frequency bandwidth XX-XX

The infrared sensors operate within the frequency bandwidth XX-XX

