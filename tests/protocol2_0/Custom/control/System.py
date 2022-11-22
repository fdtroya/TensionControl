motorTorqueConstant=2.68 #T=2.68I-0.225




#controller
timeStep=0.001
Dmotor=13.4*10**-3

Ra=8.1
La=0.28*10**-3

J=5.41*10**-8
Ke=0.0102469
Kt=10.2*10**-3 

B=3.121*10**-7

Tc=0.048
Ts=0.0536
alpha=0.002

"""
B=0.2
J=0.02
Kt=0.015
Ke=0.015
La=0.5
Ra=2
alpha=0.01
Tc=0.3
Ts=0.6
"""
constants=[B/J,Kt/J,Ra/La,Ke/La,1/La,Tc/J,(Ts-Tc)/J,alpha]
print(constants)