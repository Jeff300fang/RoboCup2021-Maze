import header
import serial
import time

def serialSetup():
        port = "/dev/ttyS0"
        rate = 9600
        comds = serial.Serial(port,rate)
        print("waiting for megapi")
        while not comds.inWaiting():
                continue
        startSign = comds.read(1)
        print("connected")
        return(comds)
def pinged():
        info = header.coms.read(1)
        return(info)

