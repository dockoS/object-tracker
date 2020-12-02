import jetson.inference
import jetson.utils

import argparse
import sys

# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.")

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# load the object detection network
net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)

# create video sources & outputs
inputs=jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv)
compteur=0
i=0
# process frames until the user exits
while True:

	# capture the next image
    img = inputs.Capture()

	# detect objects in the image (with overlay)
    detections = net.Detect(img, overlay=opt.overlay)
    compteur=compteur+len(detections)
    print("detections ")
    print(detections)    
	# print the detections
    print("detected {:d} objects in image number {:d}".format(len(detections),i))
    for detection in detections:
        print(detection)

	# render the image
    output.Render(img)

	# update the title bar
    output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

	# print out performance info
    net.PrintProfilerTimes()
	# exit on input/output EOS
    i=i+1
    if not inputs.IsStreaming() or not output.IsStreaming():
    	    break
print("compteur = {:d}".format(compteur))