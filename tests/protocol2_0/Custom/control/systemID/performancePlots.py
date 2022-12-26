import numpy as np
import matplotlib.pyplot as plt

from scipy.ndimage import uniform_filter1d


a=np.load("dataConv.npy")
a[:,2]=a[:,2]*353.5
b=np.load("dataFrConv.npy")
b[:,2]=b[:,2]*353.5

legend=["voltage","current","velocity","position"]

array=a
xAxis=array[:,-1]
for colum in [2]:#t[1]):
    yAxis=array[:,colum]
    
    y_smooth = uniform_filter1d(yAxis,size=20)
    lowess = sm.nonparametric.lowess(yAxis, xAxis, frac=0.3)
    mask=lowess[:,1]>=0
    lowess[:,1]=lowess[:,1]*mask
    plt.plot(lowess[:, 0], lowess[:, 1],label=legend[colum],linewidth=0.5)

    plt.plot(xAxis,yAxis,label=legend[colum],linewidth=0.5)
    plt.legend(loc='best')
    
#np.save("dataFiltered.npy",lowess)
plt.show()

