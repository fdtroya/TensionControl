from math import degrees, radians
import os
from threading import Thread
import keyboard

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val 
n1=0xFFFFFEBB
num=twos_comp(n1,4*8)



print(num)