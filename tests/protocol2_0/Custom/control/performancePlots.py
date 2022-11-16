import numpy as np
import matplotlib.pyplot as plt

a=np.load("dataConv.npy")
legend=["voltage","current","velocity","position"]
t=np.shape(a)
xAxis=range(t[0])
for colum in range(t[1]):
    yAxis=a[:,colum]
    plt.plot(xAxis,yAxis,label=legend[colum],linewidth=0.5)
plt.legend(loc='best')
plt.show()

