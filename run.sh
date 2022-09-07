#!/bin/bash
sudo add-apt-repository ppa:ubunutu-toolchain-r/test
sudo apt-get update
cp -avr $(pwd)/Spatial-Index-Components/include/spatialindex /usr/include
rsync -a $(pwd)/Spatial-Index-Components/lib/ /usr/lib
sudo make all
