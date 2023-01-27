import numpy as np
import matplotlib.pyplot as plt

from scipy.ndimage import uniform_filter1d



def plot(file,title,xAxisName,yAxisName):
        array=np.load(file)
        plt.title(title)
        plt.xlabel(xAxisName)
        plt.ylabel(yAxisName)
        plt.plot(array[:,0]*(5*14/360),array[:,1],label='Rigidez')
        plt.legend()
        plt.show()

def plotResponse():
    array=np.load("experimentalResponse.npy")
    plt.title("Respuesta a escalon")
    plt.xlabel("Tiempos [s]")
    plt.ylabel("Fuerza [N]")
    plt.plot(array[:,0],np.ones(np.size(array[:,0]))*0.98,label='98%')
    plt.plot(array[:,0],np.ones(np.size(array[:,0]))*1.02,label='102%')
    plt.plot(array[:,0],array[:,2],label="Motor 1")
    #plt.plot(array[:,0],array[:,4],label="Motor 2")
    plt.legend()
    plt.show()

#plot("stiffnessData.npy","Rigidez","Despalazamiento [deg]","Fuerza [N]")
plotResponse()