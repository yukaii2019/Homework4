import serial
import time
import matplotlib.pyplot as plt
import numpy as np
# XBee setting
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)
s.write("+++".encode())
char = s.read(2)
print("Enter AT mode.")
print(char.decode())
s.write("ATMY 0x165\r\n".encode())
char = s.read(3)
print("Set MY 0x165.")
print(char.decode())
s.write("ATDL 0x265\r\n".encode())
char = s.read(3)
print("Set DL 0x265.")
print(char.decode())
s.write("ATWR\r\n".encode())
char = s.read(3)
print("Write config.")
print(char.decode())
s.write("ATMY\r\n".encode())
char = s.read(4)
print("MY :")
print(char.decode())
s.write("ATDL\r\n".encode())
char = s.read(4)
print("DL : ")
print(char.decode())
s.write("ATCN\r\n".encode())
char = s.read(4)              #why not 3 ??????????????????????????
print("Exit AT mode.")
print(char.decode())
print("start sending RPC")

collect_count = np.arange(0,21,1)
t = np.arange(0,21,1)

for i in range(0,21):
    s.write("/getCollectDataTime/run\r".encode())
    char = s.read(2)
    #print(char.decode())
    collect_count[i] = int(char.decode())
    print(collect_count[i])
    #print ("aa")
    time.sleep(1)

plt.plot(t,collect_count,color = 'green',linestyle = '-',label = 'number')
plt.xlabel('Time')
plt.ylabel('number')
plt.show()
s.close()