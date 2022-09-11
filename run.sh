#!/bin/bash
echo  
echo Installing Python
echo 
sudo apt-get install python3
echo  
echo Installing Python-Pip
echo 
sudo apt-get install python3-pip
sudo pip3 install pynput
sudo pip3 install keyboard
sudo pip3 install numpy
echo  
echo Installing Spatial Index Components
echo  
sudo cp -avr $(pwd)/Spatial-Index-Components/include/spatialindex /usr/include
sudo rsync -a $(pwd)/Spatial-Index-Components/lib/ /usr/lib
echo 
echo Making Files
echo  
sudo make all
echo  
echo Setting xAuthority
echo  
xhost +si:localuser:root
echo ------------------------
echo Installation Complete
echo ------------------------
