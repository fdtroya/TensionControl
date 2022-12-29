import numpy as np
import time
import json
import matplotlib.pyplot as plt
from motor.motor import *
from scipy.signal import savgol_filter
import math


timeStep=0
dirs=[1,-1]

endTimeFr=10
#IF THE CARRIAGe CRASHES ON ONE SIDE OF THE MECHANISM DURING THE DYNAMIC TEST, REDUCE THE SIGNALFR() AMPLITUDE AND RE RUN THE TEST
def signalFr(t):# t in seconds [0 10]
    amplitude=3
    return (amplitude*math.sin((math.pi/endTimeFr)*t),0) 

# Print iterations progress
def printProgressBar (iteration, total, prefix = 'Progress', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()


def loadJson(file):
    with open(file) as json_file:
        data = json.load(json_file)
        return data
    
 
    
def post(array,plot=False):
    data=array
    omega=data[:,2]
    t=data[:,-1]
    newOmegaData=[]
    
    repeats=0
    for w_index in range(len(omega)):
        
        if w_index==0:
            newOmegaData.append([omega[w_index],t[w_index]])
        else:
            lastO,lastT=newOmegaData[-1]
            if (lastO!= omega[w_index])or repeats>=50 :
                repeats=0
                newOmegaData.append([omega[w_index],t[w_index]])
            else:
                repeats+=1

    newOmegaData=np.array(newOmegaData)
    tn=newOmegaData[:,-1]
    filteredNew=savgol_filter(newOmegaData[:,0], 5, 3)
    tn=tn.reshape((len(tn),1))
    filteredNew=filteredNew.reshape((len(filteredNew),1))
    if(plot):
        plt.plot(t,omega)
        plt.plot(tn,filteredNew)
        plt.show()
    return np.append(filteredNew,tn,axis=1)       



def dynamicTest(id,com,save=True):
    dyn1=motor(com,4000000)
    dyn1.connect([id])
    
    allData=[]#list of arrays first for positive direction, second for negative direction
    
    
    
    
    for direction in dirs:
        dyn1.homming(id,direction=direction*-1)
        dyn1.setPWMControlMode(id)
        dyn1.enableTorque(id) 
        timeLs=[]
        dirData=[]
        time.sleep(1)
        s=time.time()
        presentTime=0
        loopEndtime=time.perf_counter()
        
        while presentTime<=endTimeFr:
            loopStartTime=time.perf_counter()
            if(loopStartTime-loopEndtime>=timeStep):
                u=signalFr(presentTime)[0]*direction
                dyn1.setGoalPWM(u,id)
                data=dyn1.readBulkSensors(id)
                dirData.append(data)
                timeLs.append(presentTime)
                loopEndtime=time.perf_counter()
                presentTime=time.time()-s    
        dyn1.setGoalPWM(0,id)
        dyn1.disableTorque(id)
        data=np.array(dirData)
        timeArr=np.array(timeLs).reshape((len(timeLs),1))
        dataConv=data*motor.bulkConversion
        dataConv=np.append(dataConv, timeArr, axis=1)#format[[w0,t0],[w1,t1]...]
        allData.append(post(dataConv))
        dyn1.disableTorque(id)
    
    i=0
    if(save):
        for direction in dirs:
            np.save("dataFrFiltered"+str(direction)+".npy",allData[i]) 
            i+=1
    return allData

def stableTest(id,com,MaxOmega,NDataPoints,plot=False,save=True):
    tol=0.023980823895/2
    lowerL=MaxOmega*0.5
    upperL=MaxOmega*0.85
    lowerOmegaRange=np.arange(0,lowerL,lowerL/(NDataPoints*2))
    upperOmegaRange=np.arange(upperL,MaxOmega,(MaxOmega-upperL)/NDataPoints)
    ranges=[lowerOmegaRange,upperOmegaRange]
    dyn1=motor(com,4000000)
    dyn1.connect([id])
    dyn1.disableTorque(id)

    dyn1.homming(id)

    dyn1.setExtendedPositionControlMode(id)
    dyn1.enableTorque(id)
    dyn1.setGoalPositionAngle(10/(14*5/360),id)
    time.sleep(5)
    dyn1.disableTorque(id)
    dyn1.setVelocityControlMode(id)
    dyn1.enableTorque(id)
    data=loadJson("models.json")
    kt=data["motor"]["Kt"]
    allData=[]#list points[[friction Torque0,Omega0],[friction Torque1,Omega1]....] for positive and negative directions 
    leng=(len(ranges[0])+len(ranges[1]))*2
    progress=1
    for omegaRange in ranges:
        for omega in omegaRange:
            for direction in dirs:
                omegaData=[]#array samples of same omega and Torque [[friction Torquei,Omegai],[friction Torquei,Omegai]]
                presentTime=0
                
               
                pos=dyn1.getPresentPositionAngle(id)*(14/360)*5
                loopEndtime=time.perf_counter()
                dyn1.setGoalVelocity(omega*direction,id)
                time.sleep(1.5)
                s=time.time()
                while(((direction>0 and pos<=50)or (direction<0 and pos >5))and presentTime<=5):
                    loopStartTime=time.perf_counter()
                    if(loopStartTime-loopEndtime>=timeStep):
                        data=dyn1.readBulkSensors(id)*motor.bulkConversion
                        presentOmega=data[2]#rad/s
                        i=data[1]#mAmp
                        if((presentOmega-tol<=omega*direction<=presentOmega+tol)and i!=0):
                            omegaData.append([i,presentOmega])
                        pos=data[-1]*(14/360)*5#mm
                        loopEndtime=time.perf_counter()
                        presentTime=time.time()-s
                if(len(omegaData)>0):
                    omegaData=np.array(omegaData)*np.array([0.001*kt,1])#Conversion to amps and Torque
                    meanData=np.mean(omegaData,axis=0)#array[meanTorque,Omega]
                    
                    allData.append(np.array(meanData))
                printProgressBar(progress,leng)
                progress+=1
    allData=np.array(allData)
    dyn1.disableTorque(id)
    if(plot):
        plt.plot(allData[:,1],allData[:,0],'.')
        plt.grid(axis='x', color='0.95')
        plt.grid(axis='y', color='0.95')
        plt.show()
    if (save):
        np.save("stableStateData.npy",allData)
    return allData
                        







   

    

if __name__ == '__main__':  
    #dynamicTest(1,'COM3')
    stableTest(1,'COM3',2.5,25,True,True)