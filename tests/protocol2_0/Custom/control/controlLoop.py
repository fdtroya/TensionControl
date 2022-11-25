



class loop(object):


    def __init__(self,motorController,sensor,frequency):
        self.motorController=motorcontroller
        self.sensor=sensor
        self.freq=frequency
        self.constants=constants
        self.gains=gains

    def Tm(omega,Tfr,Ftd):
        self.con