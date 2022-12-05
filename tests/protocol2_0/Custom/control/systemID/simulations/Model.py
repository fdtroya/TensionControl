class Model(object):


    def __init__(self,stateEq):
        self.stateEq=stateEq
        self.stateVarLog=[]
    

    def evalStateEq(self,stateVar):
        stateVarDot=[]
        c=0
        for eq in self.stateEq:
            stateVarDot.append(eq(stateVar[0]))
            c+=1
        self.stateVarLog.append(stateVar)
        return stateVarDot


    