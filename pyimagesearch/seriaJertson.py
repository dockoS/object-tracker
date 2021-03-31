import serial
import time
arduino=serial.Serial(port="/dev/ttyACM0",
baudrate=115200
)
while True:
	try:
		arduino.write("hello IRD !".encode())
		data=arduino.readline()
		if data:
			print(data)
		time.sleep(1)
	except Exception as e:
		print(e)
		arduino.close()




















