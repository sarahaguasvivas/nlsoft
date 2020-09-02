from gym.block_gym import *
import time
from datetime import date
import traceback, sys
B = BlockGym()
B.reset()
today = date.today()
f = open("data_" + today.strftime("%b_%d_%Y_8")+".txt", "w+")
print(B.get_state())
