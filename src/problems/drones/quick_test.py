import logging
import sys
import time
import math
import numpy as np

import os
from random import seed
from random import randint
import subprocess
from multiprocessing import Process, Value, Array, Event

# pos = np.zeros([3, 4])
pos = np.array([[1,2,3],[4,5,6],[7,8,9]])
print(pos)
print(np.amin(pos[:,0]))

for drone in range(5):
	print(drone)
print('break')
print(3 not in [1,2,3])