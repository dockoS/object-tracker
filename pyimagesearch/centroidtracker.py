# import the necessary packages
from scipy.spatial import distance as dist
from scipy.spatial import distance_matrix as dist_mat
from collections import OrderedDict
import numpy as np
import pandas as pd
from math import sqrt
import time
def distance(p):
    return sqrt(p[0]**2 +p[1]**2)

def signe(speed1,speed2):
    return (speed1[0]*speed2[0]>=0)
class CentroidTracker():
	def __init__(self, maxDisappeared=50):
		# initialize the next unique object ID along with two ordered
		# dictionaries used to keep track of mapping a given object
		# ID to its centroid and number of consecutive frames it has
		# been marked as "disappeared", respectively
		self.nextObjectID = 0
  		#the dictionnary to save the rate variations of the objects .This allow to know if the objets is stopped or not
		self.variation_rates_centroids=OrderedDict()
		#dictionaries used to keep track of mapping a given object
		self.objects = OrderedDict()
		self.disappeared = OrderedDict()
        #It save the class of each object
		self.classes=OrderedDict()
		# The time in milliseconde .
		self.temps=time.time()*1000
		# the time t or the periode, it allow to calculate the acceleration or velocity
		self.t=0
		# Save the instantaneous speed of each object in each image
		self.speed_vectors_dict=OrderedDict()
		# Save the instantaneous speed of each object in each image
		self.distanceInterVehicule=OrderedDict()
		self.numberImages=0
		# store the number of maximum consecutive frames a given
		# object is allowed to be marked as "disappeared" until we
		# need to deregister the object from tracking
		self.maxDisappeared = maxDisappeared
    #This Methode save a centroid  and initialize the information of  a centroid  
	def register(self, centroid,Class):
		# when registering an object we use the next available object
		# ID to store the centroid
		self.objects[self.nextObjectID] = centroid
		self.speed_vectors_dict[self.nextObjectID]=list()
		self.variation_rates_centroids[self.nextObjectID]=0
		self.disappeared[self.nextObjectID] = 0
		self.classes[self.nextObjectID]=Class
		self.nextObjectID += 1

	def deregister(self, objectID):
		# to deregister an object ID we delete the object ID from
		# both of our respective dictionaries
		
		del self.objects[objectID]
		del self.classes[objectID]
		del self.speed_vectors_dict[objectID]
	#speed calculate the velocity on the other hand the acceleration
	def speed(self,centroidA,centroidB,t_i_1):
		tA=self.t
		vx=0
		vy=0
		if tA!=t_i_1 :
			vx=(centroidA[0]-centroidB[0])/(tA-t_i_1)
			vy=(centroidA[1]-centroidB[1])/(tA-t_i_1)
		
		return {"speed":(vx,vy),"time":(tA,t_i_1)}
    # this calculate the interdistance of all objects 
	def interdistance(self):
		mat=[]
		for object in self.objects:
			mat.append(self.objects[object])	
		return dist_mat(mat,mat)
    # The most important method .All happen in this method
	def update(self, rects):
    	
		# check to see if the list of input bounding box rectangles
		# is empty
		if len(rects) == 0:
			# return early as there are no centroids or tracking info
			# to update
			return self.objects

		# initialize an array of input and class centroids  for the current frame
		inputCentroids = np.zeros((len(rects), 2), dtype="float")
		inputClasses = np.zeros((len(rects)), dtype="int")

		# loop over the bounding box rectangles
		for i in range(0,len(rects)):
			# use the bounding box coordinates to derive the centroid
			inputCentroids[i] = rects[i].Center
			inputClasses[i]=rects[i].ClassID
		
		# if we are currently not tracking any objects take the input
		# centroids and register each of them
		if len(self.objects) == 0:
			for i in range(0, len(inputCentroids)):
				self.register(inputCentroids[i],inputClasses[i])

		# otherwise, are are currently tracking objects so we need to
		# try to match the input centroids to existing object
		# centroids
		else:
			# grab the set of object IDs and corresponding centroids
			objectIDs = list(self.objects.keys())
			objectCentroids = list(self.objects.values())

			# compute the distance between each pair of object
			# centroids and input centroids, respectively -- our
			# goal will be to match an input centroid to an existing
			# object centroid
			D = dist.cdist(np.array(objectCentroids), inputCentroids)

			# in order to perform this matching we must (1) find the
			# smallest value in each row and then (2) sort the row
			# indexes based on their minimum values so that the row
			# with the smallest value as at the *front* of the index
			# list
			rows = D.min(axis=1).argsort()

			# next, we perform a similar process on the columns by
			# finding the smallest value in each column and then
			# sorting using the previously computed row index list
			cols = D.argmin(axis=1)[rows]

			# in order to determine if we need to update, register,
			# or deregister an object we need to keep track of which
			# of the rows and column indexes we have already examined
			usedRows = set()
			usedCols = set()

			# loop over the combination of the (row, column) index
			# tuples
			#TB is the time now in seconde 
			tB=time.time()*1000
			
			
			for (row, col) in zip(rows, cols):
				# if we have already examined either the row or
				# column value before, ignore it
				# val
				if row in usedRows or col in usedCols:
					continue

				# otherwise, grab the object ID for the current row,
				# set its new centroid, and reset the disappeared
				# counter
				objectID = objectIDs[row]
				#compute the euclidien distance between  the origin and a centroid
				distance0=distance(self.objects[objectID])
				distance1=distance(inputCentroids[col])
				
				distanceSeries=pd.Series([distance0,distance1])
				#data contains the variation rate between the previous position and the current position
				data=distanceSeries.pct_change()[1]
				#this line update the sum of the object's variation rate
				self.variation_rates_centroids[objectID]=self.variation_rates_centroids[objectID]+data
				print("variation")
				print(data)
				print("somme des variations")
				print(self.variation_rates_centroids[objectID])
				print("centroid")
				print(objectID)
				# lets compute the speed of each centroid updated
				speed_vector=self.speed(self.objects[objectID],inputCentroids[col],self.t+tB-self.temps)
				print(speed_vector)
			    #after calculating the velocity, lets append the new speed of the centroid  
				self.speed_vectors_dict[objectID].append(speed_vector)
				#Lets update the centroid value
				#before updating the centroid we must check if it is the same centroid: If the centroid variation is
				#greater than 0.2 (bias) we assume that this centroid is a new centroid an unseen object and we register it
				if len(self.speed_vectors_dict[objectID])>=2:	
					if abs(data)<0.3 and signe(self.speed_vectors_dict[objectID][-2]["speed"],speed_vector["speed"])>0:
						self.objects[objectID] = inputCentroids[col]
						self.disappeared[objectID] = 0
					else:
						self.register(inputCentroids[col],inputClasses[col])
				else:
					if abs(data)<0.3:
						self.objects[objectID] = inputCentroids[col]
						self.disappeared[objectID] = 0
					else:
						self.register(inputCentroids[col],inputClasses[col])
				# indicate that we have examined each of the row and
				# column indexes, respectively
				usedRows.add(row)
				usedCols.add(col)
			print(self.objects)
			self.t=self.t + tB-self.temps
			self.temps=tB
			# compute both the row and column index we have NOT yet
			# examined
			unusedRows = set(range(0, D.shape[0])).difference(usedRows)
			unusedCols = set(range(0, D.shape[1])).difference(usedCols)

			# in the event that the number of object centroids is
			# equal or greater than the number of input centroids
			# we need to check and see if some of these objects have
			# potentially disappeared
			if D.shape[0] >= D.shape[1]:
				# loop over the unused row indexes
				for row in unusedRows:
					# grab the object ID for the corresponding row
					# index and increment the disappeared counter
					objectID = objectIDs[row]
					self.disappeared[objectID] += 1

					# check to see if the number of consecutive
					# frames the object has been marked "disappeared"
					# for warrants deregistering the object
					#if self.disappeared[objectID] > self.maxDisappeared:
						#self.deregister(objectID)

			# otherwise, if the number of input centroids is greater
			# than the number of existing object centroids we need to
			# register each new input centroid as a trackable object
			else:
				for col in unusedCols: 
					self.register(inputCentroids[col],inputClasses[col])

		# return the set of trackable objects
		if len(self.objects)>=2:
			# if the number of object is greater than 1 we compute the 
    		# distance matrix and save it in the distanceInterVehicule's dictionnary
			# We use the number of images to know what is the distance between vehicles in each image
			self.distanceInterVehicule[self.numberImages]=self.interdistance()
			print("mattttttttttttttttttttttt")
			print(self.distanceInterVehicule)
		self.numberImages=self.numberImages+1
		return  self.objects
		
	#This method delete the the vehicles stopped
	def filtrage(self):
		print("avant")
		print(self.objects)
		print(self.variation_rates_centroids)
		print(self.classes)
		for i in range(len(self.objects)):
			if abs(self.variation_rates_centroids[i])<0.1:
				self.deregister(i)
		print("Apres")
		print(self.objects)
		print(self.classes)
		print(self.variation_rates_centroids)
		print("Speed vectors")
		# print(self.speed_vectors_dict)
		return len(self.objects)