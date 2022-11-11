from math import degrees, radians
import os
from threading import Thread
import keyboard



num=0xFFFFFEBB
absVal= num&0x7FFFFFFF
sign=(num>>31)
if(sign):
    inv=~absVal
    preval=inv*-1
    val1= preval-1
else:
    val2=absVal



val=num.to_bytes(4,byteorder="big")
final=int.from_bytes(val,byteorder="big",signed=True)
num2=num


print(num)