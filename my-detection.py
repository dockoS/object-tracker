import jetson.inference
import jetson.utils
from pyimagesearch.centroidtracker import CentroidTracker
import argparse
import pandas as pd
import sys
from collections import OrderedDict
from matplotlib import pyplot as plt
from time import perf_counter
# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.")

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.7, help="minimum detection threshold to use") 

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)
#instance ct 
ct=CentroidTracker()
# load the object detection network
net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)

# create video sources & outputs
variation=[]
variation1=[]
time_start=perf_counter()
inputs=jetson.utils.videoSource(opt.input_URI, argv=['--input-codec=h264'])
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv)
compteur=0
i=0
data=[]
#the index of vehicles in labels which is trained the model
labels=[3,5,6,7,8]
#labels=[2,4,3,5,6,7,8]
# process frames until the user exits
while True:
	# capture the next image
	img = inputs.Capture()
	print("la width {:f}".format(img.width))
	# detect objects in the image (with overlay)
	detections = net.Detect(img, overlay=opt.overlay)
	rects=[]
	compteur=compteur+len(detections)
	print("detections ={:d}".format(len(detections)))    
	# print the detections
	#print("detected {:d} objects in image number {:d}".format(len(detections),i))
	#In this loop we append in the rects the information of each detection
	for detection in detections:
		if detection.ClassID in labels:
			# print(" class == {:d}  confiance={:f}". format(detection.ClassID,detection.Confidence))
			# print("center= "+str(detection.Center))
			rects.append(detection)
			data.append({"x":detection.Center[0],"y":detection.Center[1]})
	#print("taille rectangle =={:d}".format(len(rects)))
	# render the image
	#call the update method with rects
	objects=ct.update(rects)
	#print("objets traqueeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeees =={:d}".format(len(objects)))
	output.Render(img)
	# update the title bar
	output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))
	time_current=perf_counter()
	if (time_start-time_current>30):
		#ct.save_data_to_csv()
		time_start=time_current
		i=i+1
		print("DOCKOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
	# print out performance info
	net.PrintProfilerTimes()
	# exit on input/output EOS
	if not inputs.IsStreaming() or not output.IsStreaming():
			break
#print("nombre d objet final= {:d}".format(ct.filtrage()))
print(i)
