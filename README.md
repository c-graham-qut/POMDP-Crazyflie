# Implementation of POMDP using Bitcraze Crazyflie Drones
This repository is set up to develop the necessary software to implement a POMDP model using the Bitcraze Crazyflie Drones for a thesis topic at the Queensland Univeristy of Technology (QUT). 

This repository is meant for complete beginners in order for future students to focus on researching POMDP problem definitions without having to have extensive knowledge of linux, git or coding. 

--------------------------------------------------------------------------
## Requirements

### Software
1. [Ubuntu v20.04](http://www.releases.ubuntu.com/20.04)
2. [TAPIR Toolkit and Dependancies](https://github.com/RDLLab/tapir)
3. Bitcraze Crazyflie Client and Libraries
4. Bitcraze Crazyflie v2.1 - Firmware Pack XXXX

### Hardware
1. [Laptop](https://www.dell.com/en-au/work/shop/2-in-1-laptops-tablet-pcs/latitude-7390-2-in-1/spd/latitude-13-7390-2-in-1-laptop)
2. [Bitcraze Crazyflie Drones v2.1](https://store.bitcraze.io/products/crazyflie-2-1)
3. [Bitcraze Lighthouse Positioning Decks](https://www.bitcraze.io/products/lighthouse-positioning-deck)
4. [Bitcraze CrazyRadio Dongle](https://store.bitcraze.io/collections/accessories/products/crazyradio-pa)
5. [Infrared Positioning Systems](https://store.bitcraze.io/collections/positioning/products/lighthouse-v2-base-station)
6. [Lighhouse Basestation Stands](https://www.amazon.com.au/Selens-Adjustable-Aluminium-Stands-Carrying/dp/B01N7QR332/ref=d_pd_sim_sccl_2_5/356-0311129-5408244?pd_rd_w=64CKI&content-id=amzn1.sym.128b624f-6806-46cb-b7e9-12435bd6f216&pf_rd_p=128b624f-6806-46cb-b7e9-12435bd6f216&pf_rd_r=FQKJ0ASHG5JY35GA7B5P&pd_rd_wg=FtWAi&pd_rd_r=696c03ca-82fc-4803-bb4e-034f009a768d&pd_rd_i=B01N7QR332&psc=1)

## Toolkit Implementation

The toolkit requries some set-up in order for the drones to function appropriately. The implementation methodlogy is split into two main sections; software and hardware. The software implementation goes through all necessary steps to install the correct libraries, depndencies and configures your ground control station with the appropriate authorisations.


### Software
The following procedure will demonstrate how to install this software toolkit.
#### Step 1 - Install Ubuntu v20.04

#### Step 2 - Download POMDP-Crazyflie Software Toolkit
##### Step 2.1 - Open a new terminal window.
##### Step 2.2 - Navigate to the repository location.
##### Step 2.3 - Initiate the run file:
The 'run.sh' file downloads the necessary repositories for the POMDP TAPIR toolkit and the Bitcraze Crazyflies. 

First you will have to initiate the run file. Run the following code in the open terminal:
```
sudo chmod +x run.sh
```

Then, run the following code in the open terminal:
```
./run.sh
```

###### Run File Contents
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

##### Step 2.4 - Wait for the installation to complete
The installation of the toolkits, repositories and compilation will take a few minutes. 

Once it is complete the following should be seen in the terminal window:
```
---------------------
Installation Complete
---------------------
```

##### Step 3 - Install Bitcraze Crazyflie Client and Libraries
Using the same open terminal you will download the necessary libaries and files for the Bitcraze Crazyflie Client and Libraries.

###### Install Client and Libraries
Install [cflib and cfclient](https://github.com/bitcraze/crazyflie-clients-python/blob/master/docs/installation/install.md)

###### Install Radio Dongle Support Software
Install [CrazyRadio Support Software](https://github.com/bitcraze/crazyradio-firmware)

### Hardware 

#### Laptop or Desktop

#### Bitcraze
##### Crazyflie v2.1

##### Lighthouse Positioning Sensors


