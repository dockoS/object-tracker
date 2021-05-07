import serial
import time
import pandas as pd
arduino = serial.Serial(port='/dev/ttyACM0',baudrate=115200, timeout=1)
df=pd.read_csv("info_objets_n°0.csv")
print(df.shape)
try:
	for i in range(df.shape[0]):
		text=df.iloc[i]["mobilite"]+","+str(df.iloc[i]["taux_de_variation"])+","+str(df.iloc[i]["type"])+"!"
		arduino.write(text.encode())
		data = arduino.readline()
		if(data):
			print(data)
		time.sleep(1)
		print("Hi arduino")
except:
	arduino.close()
	