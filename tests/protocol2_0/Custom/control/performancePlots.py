import numpy as np
import matplotlib.pyplot as plt

a=np.load("dataConv.npy")
time=np.load("timeStep.npy")
legend=["voltage","current","velocity","position"]
t=np.shape(a)
xAxis=time
for colum in range(2):#t[1]):
    yAxis=a[:,colum]
    plt.plot(xAxis,yAxis,label=legend[colum],linewidth=0.5)
    plt.legend(loc='best')
    
plt.show()

