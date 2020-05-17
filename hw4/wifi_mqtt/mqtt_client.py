import paho.mqtt.client as paho
import matplotlib.pyplot as plt
import numpy as np
import time

# https://os.mbed.com/teams/mqtt/wiki/Using-MQTT#python-client
# MQTT broker hosted on local machine
mqttc = paho.Client()
# Settings for connection
# TODO: revise host to your ip
host = "192.168.1.136"
topic = "Mbed"
# Callbacks
t = np.arange(0,40,0.1) 
X = np.arange(0,40,0.1)
Y = np.arange(0,40,0.1)
Z = np.arange(0,40,0.1)
m = 0
j = 0
def on_connect(self, mosq, obj, rc):
      print("Connected rc: " + str(rc))
def on_message(mosq, obj, msg):
      global m,t,X,Y,Z,j
      print("[Received] Topicddd: " + msg.topic + ", Message: " + str(msg.payload,encoding = "utf-8") + "\n")
      passNum = float(str(msg.payload,encoding = "utf-8").rstrip(b'\x00'.decode()))
      print(passNum)
      if passNum==99999:
            print("disconnect")
            mqttc.disconnect()
      elif m == 0:
            X[j] = passNum
            m = 1
      elif m == 1:      
            Y[j] = passNum
            m = 2
      elif m == 2:
            Z[j] = passNum
            m = 3       
      elif m == 3:
            t[j] = passNum/1000
            m = 0
            j = j+1
      

def on_subscribe(mosq, obj, mid, granted_qos):
      print("Subscribed OK")


def on_unsubscribe(mosq, obj, mid, granted_qos):
      print("Unsubscribed OK")


# Set callbacks

mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe


# Connect and subscribe

print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)

mqttc.loop_forever()
print(j)
for k in range(0,j):
      print("x = %f y = %f z = %f t = %f"%(X[k],Y[k],Z[k],t[k]))
plt.plot(t[0:j],X[0:j],color = 'green',linestyle = '-',label = 'x')
plt.plot(t[0:j],Y[0:j],color = 'red',linestyle = '-',label = 'y')
plt.plot(t[0:j],Z[0:j],color = 'blue',linestyle = '-',label = 'z')
plt.legend(loc = 'lower left')
plt.xlabel('timestamp')
plt.ylabel('acc value')
plt.show()

