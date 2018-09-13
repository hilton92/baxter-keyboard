#Author: Benjamin Hilton
#ME 537 Project
#Date: December 2017
#Title: Baxter_server_ME537

#import dependencies
import numpy as np
import cv2
import sys
import socket
import time

#Prompt the user for the number of characters to type.
chars = raw_input("Enter the number of characters to type:")

#Initialize the looping variable to one.
loop = 1;

#Socket Setup - socket was used because the client script needs libraries of opencv that were not installed.
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = socket.gethostname()
port = 9547
s.bind(('', port))
s.listen(5)

#Wait until a connection request is received, then proceed.
clientSocket, addr = s.accept()
print("got a connection from %s" % str(addr))


tracker = cv2.TrackerKCF_create() #create tracker object
cap = cv2.VideoCapture(1) #load video (0 or 1 for camera device number)
ok, frame = cap.read() #read one frame of the stream
listofbbox = [] #initialize the vector of bbox lists

#Setup code so that the output can be saved to file.
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output12_10.avi', fourcc, 20.0, (640,480))

#Select the bbox for each character in order. Press enter after each one.
for i in range(0, int(chars)):
	listofbbox.append(cv2.selectROI(frame, False)) #select location of ROI (region of interest)

#Set bbox equal to the first bbox specified.
bbox = listofbbox[0]
ok = tracker.init(frame, bbox) #initialize tracker




while(True):
	ok, frame = cap.read() #read next image of stream
	ok, bbox = tracker.update(frame) #update location of bbox

	if ok:
		p1 = (int(bbox[0]), int(bbox[1]))
        	p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        	cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1) #add bbox rectangle
		cv2.imshow('frame', frame)
		#Send information of bbox to the client computer.
		clientSocket.send(str(int(bbox[0])).zfill(3) + '-' + str(int(bbox[1])).zfill(3) + '-' + str(int(bbox[2])).zfill(3) + '-' + str(int(bbox[3])).zfill(3))
	out.write(frame)

	# if 'n' is pressed, move on to the next character and next bbox
	if cv2.waitKey(1) & 0xFF == ord('n'):
		if loop >= int(chars):
			print('exiting')
			break
		bbox = listofbbox[loop]
		tracker = cv2.TrackerKCF_create()
		ok = tracker.init(frame, bbox)
		loop = loop + 1
		print('next')



#close all windows
cap.release()
out.release()
cv2.destroyAllWindows()



