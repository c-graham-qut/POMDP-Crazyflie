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

## Software Toolkit
For more information on the POMDP model used please visit -> [TAPIR POMDP - Master](https://github.com/RDLLab/tapir)

### Creators
TAPIR [1] is a C++ implementation of the Adaptive Belief Tree (ABT) algorithm [2]. ABT is an online POMDP solver capable of adapting to modifications to the POMDP model without the need to reconstruct the policy from scratch. For continuous action spaces, TAPIR also includes an implementation of General Pattern Search in Adaptive Belief Trees (GPS-ABT) [3].

[1] D. Klimenko and J. Song and. H. Kurniawati. TAPIR: A Software Toolkit for Approximating and Adapting POMDP Solutions Online. Proc. Australasian Conference on Robotics and Automation. 2014. [pdf]

[2] H. Kurniawati and V. Yadav. An Online POMDP Solver for Uncertainty Planning in Dynamic Environment. Proc. Int. Symp. on Robotics Research. 2013. [pdf]

[3] K. Seiler and H. Kurniawati and S.P.N. Singh. An Online and Approximate Solver for POMDPs with Continuous Action Space. Proc. IEEE Int. Conference on Robotics and Automation (ICRA). 2015. [pdf]

For the latest news, please visit the TAPIR website.

## Toolkit Implementation

### Installation
Download the entire repository into a folder of your choise.
Open a 
