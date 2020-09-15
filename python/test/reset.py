import os, sys
sys.path.append(os.path.join(os.environ['HOME'], 'gpc_controller'))

from block_gym.block_gym import *
import time
from datetime import date
import traceback, sys
B = BlockGym()
B.reset()
B.step([-80., -50.])
today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y_8")+".txt", "w+")
print(B.get_state())
