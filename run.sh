#!/bin/bash
sudo add-apt-repository ppa:ubunutu-toolchain-r/test
sudo apt-get update
sudo apt-get install python3
sudo apt install git python3-pip libxcb-xinerama0
sudo pip3 install --upgrade pip
sudo pip3 install pynput
sudo pip3 install keyboard
sudo pip3 install numpy
sudo pip3 install cflib
cp -avr $(pwd)/Spatial-Index-Components/include/spatialindex /usr/include
rsync -a $(pwd)/Spatial-Index-Components/lib/ /usr/lib
sudo make all
sudo git clone https://github.com/bitcraze/crazyflie-clients-python
sudo cd crazyflie-clients-python
sudo cd
xhost +si:localuser:root
sudo pip3 install -e .

