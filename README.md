# Implementation of POMDP using Bitcraze Crazyflie Drones
This repository is set up to develop the necessary software to implement a POMDP model using the Bitcraze Crazyflie Drones for a thesis topic at the Queensland Univeristy of Technology (QUT). 

## Requirements
### Software
1. Ubuntu v20.04
2. TAPIR Toolkit and Dependancies
3. Bitcraze Crazyflie Client and Libraries
4. Bitcraze Crazyflie v2.1 - Firmware Pack XXXX

### Hardware
1. Laptop or Desktop (Running Ubuntu v20.04
2. [Bitcraze Crazyflie Drones v2.1](https://store.bitcraze.io/products/crazyflie-2-1)
3. [Bitcraze Lighthouse Positioning Decks](https://www.bitcraze.io/products/lighthouse-positioning-deck)
4. [Bitcraze CrazyRadio Dongle](https://store.bitcraze.io/collections/accessories/products/crazyradio-pa)
5. [Infrared Positioning Systems](https://store.bitcraze.io/collections/positioning/products/lighthouse-v2-base-station)
6. [Lighhouse Basestation Stands](https://www.amazon.com.au/Selens-Adjustable-Aluminium-Stands-Carrying/dp/B01N7QR332/ref=d_pd_sim_sccl_2_5/356-0311129-5408244?pd_rd_w=64CKI&content-id=amzn1.sym.128b624f-6806-46cb-b7e9-12435bd6f216&pf_rd_p=128b624f-6806-46cb-b7e9-12435bd6f216&pf_rd_r=FQKJ0ASHG5JY35GA7B5P&pd_rd_wg=FtWAi&pd_rd_r=696c03ca-82fc-4803-bb4e-034f009a768d&pd_rd_i=B01N7QR332&psc=1)

## Overview
This toolkit is being used as it provides a framework that can be integrated with the Bitcraze Crazyflie drones. 

### TAPIR Software Toolkit
For more information on the POMDP model used please visit -> [TAPIR POMDP - Master](https://github.com/RDLLab/tapir)

### Software Modifications
In order for the TAPIR POMDP toolkit to be implemented with the Crazyflie drones, modifications to some of the files was necessary to integrate the observations and movements. 

These modifications were made by Marc Schneider for his masters report titled 'XXX'.

The files that were modification include:

### POMDP Problem Definition
For this project the POMDP Problem Definition is illustrated in the image below: 


## Toolkit Implementation

### Installation
The following procedure will demonstrate how to install this software toolkit.

#### Step 1 - Download the entire repository into a folder of your choise.
#### Step 2 - Open a new terminal window.
#### Step 3 - Navigate to the repository location.
#### Step 4 - Initiate the run file:
The 'run.sh' file downloads the necessary repositories for the POMDP TAPIR toolkit and the Bitcraze Crazyflies. 

First you will have to initiate the run file. Run the following code in the open terminal:
```
sudo chmod +x run.sh
```

Then, run the following code in the open terminal:
```
./run.sh
```

##### Run File Contents
The file includes the following commands: 

The code below downloads and installs python, pip, pynput, keyboard and numpy repositories.
```
sudo apt-get install python3
sudo apt-get install python3-pip
sudo pip3 install pynput
sudo pip3 install keyboard
sudo pip3 install numpy
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

The code below sets XAuthority for all users on the device in order to connect to the display (:0).
```
xhost +si:localuser:root
```

#### Step 5 - Wait for the installation to complete
The installation of the toolkits, repositories and compilation will take a few minutes. 

Once it is complete the following should be seen in the terminal window:
```
---------------------
Installation Complete
---------------------
```

#### Step 6 - Install Bitcraze Crazyflie Client and Libraries
Using the same open terminal you will download the necessary libaries and files for the Bitcraze Crazyflie Client and Libraries.

6.1 Install [cflib and cfclient](https://github.com/bitcraze/crazyflie-clients-python/blob/master/docs/installation/install.md)
6.2 Install [CrazyRadio Support Software]()


### Hardware Set-Up

#### Laptop or Desktop

#### Bitcraze
##### Crazyflie v2.1

##### Lighthouse Positioning Sensors

##### 

## General Information

### Communication 
The Crazyflie radio dongles operate within the frequency bandwidth XX-XX

The infrared sensors operate within the frequency bandwidth XX-XX

