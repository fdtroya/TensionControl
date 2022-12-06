from dataCollection import signal
import math 
import numpy as np
import matplotlib.pyplot as plt

def sensorSim(omega):
    return (omega//0.02391)*0.02391


def sign(n):
    return int(n>0) - int(n<0)
    
def runSim(listK,timeStep,endTime):

    return uL,iL[1:],omegaL[:-1],omegaDot


