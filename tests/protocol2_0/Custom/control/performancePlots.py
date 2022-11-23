import numpy as np
import matplotlib.pyplot as plt
from statsmodels.nonparametric.kernel_regression import KernelReg
a=np.load("dataConv.npy")
a[:,2]=a[:,2]*353.5
b=np.load("dataBConv.npy")
b[:,2]=b[:,2]*353.5
time=np.load("timeStep.npy")
legend=["voltage","current","velocity","position"]
t=np.shape(a)
xAxis=time

for colum in [2]:#t[1]):
    yAxis=b[:,colum]

    plt.plot(xAxis,yAxis,label=legend[colum],linewidth=0.5)

    plt.legend(loc='best')
    
plt.show()

