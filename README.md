# Implementation of POMDP using Bitcraze Crazyflie Drones
This repository is set up to develop the necessary software to implement a POMDP model using the Bitcraze Crazyflie Drones for a thesis topic at the Queensland Univeristy of Technology (QUT). 

This repository is meant for complete beginners in order for future students to focus on researching POMDP problem definitions without having to have extensive knowledge of linux, git or coding. 

This 'readme.md' document only explains how to install this softare toolkit. 

Please navigate to the [wiki](https://github.com/c-graham-qut/POMDP-Crazyflie/wiki) for more information about this project.

## Installation
The implementation methodlogy is split into two main sections; software and hardware. The software implementation goes through all necessary steps to install the correct libraries, depndencies and configures your ground control station with the appropriate authorisations. 

### Software
The following procedure will demonstrate how to install this software toolkit.

#### Step 1 - Install Ubuntu v20.04
The operating system for your laptop or dekstop must be Ubuntu v20.04. No other Ubuntu version has been tested with this software toolkit. 

- Download Ubuntu v20.04 -> [here](http://www.releases.ubuntu.com/20.04)
- Create a bootable USB device. This project used [Rufus](https://rufus.ie/en/) to create the bootable media device. To create the bootable media device, this project used [How-To Geek: Create Bootable USB Device](https://www.howtogeek.com/howto/linux/create-a-bootable-ubuntu-usb-flash-drive-the-easy-way/) for this process.
- Boot your computer from the USB Drive. You will have to restart your computer, with the bootable media device inserted, and access your computers boot menu. From this boot menu you will be able to select the bootable media device to install the linux distribution. Note: you will have to find the appropriate methodology for your current operating system (Windows, Linux or Mac) as this process can vary.

#### Step 2 - Download POMDP-Crazyflie Software Toolkit
This step will detail the steps to install this software toolkit only. Some steps reference Bitcraze 
- Download this toolkit as a zip file. 
- Extract the zip file in a location of your choice.
- Open a new terminal window within the extracted folder. Right-Click within the folder and select 'Open in Terminal'. 

##### Initiate the run file:
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

##### Wait for the installation to complete
The installation of the toolkits, repositories and compilation will take a few minutes. 

Once it is complete the following should be seen in the terminal window:
```
---------------------
Installation Complete
---------------------
```

##### Step 3 - Install Bitcraze Crazyflie Client and Libraries
Using the same open terminal you will download the necessary libaries and files for the Bitcraze Crazyflie Client and Libraries.

- Install [cflib and cfclient](https://github.com/bitcraze/crazyflie-clients-python/blob/master/docs/installation/install.md)

- Install [CrazyRadio Support Software](https://github.com/bitcraze/crazyradio-firmware)



