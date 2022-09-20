#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import time
import cv2
#import imutils
from imutils.video import FPS
from imutils.video import VideoStream
# Importing Libraries
import serial
import time
#from serial import Serial


INPUT_FILE='Plant_video.mp4'
OUTPUT_FILE='output.avi'
CONFIG_FILE='yolov4-tiny.cfg'
WEIGHTS_FILE='weight/Yolov4_Lettuce_tiny_2000.weights'
CONFIDENCE_THRESHOLD=0.70

H=None
W=None

fps = FPS().start()

fourcc = cv2.VideoWriter_fourcc(*"MJPG")
writer = cv2.VideoWriter(OUTPUT_FILE, fourcc, 30,
	(800, 600), True)

LABELS = ['Lettuce']

np.random.seed(4)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")


net = cv2.dnn.readNetFromDarknet(CONFIG_FILE, WEIGHTS_FILE)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
#vs = cv2.VideoCapture(INPUT_FILE)
url = "http://192.168.0.101:8080/video"

vs = cv2.VideoCapture(0)
#vs.set(3,800)
#vs.set(4, 600)
vs.set(10,100)
vs.set(28, 0) 
a=1
#brightness=50
#vs.set(cv2.CAP_PROP_BRIGHTNESS, 1500)
# determine only the *output* layer names that we need from YOLO
ln = net.getLayerNames()
ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]


# In[2]:


ser = serial.Serial('COM5', 9600)


# In[3]:


cnt=0
#brightness=500
#CONFIDENCE_THRESHOLD=0.6
P_MT=0
P_ST=0
P_RT=0
while True:
    key=cv2.waitKey(1)
    cnt+=1
    #print ("Frame number", cnt)
    try:
        (grabbed, image) = vs.read()
    except:
        break
    #norm_img=np.zeros((256,256))
    #image=cv2.normalize(image,  norm_img, 0, 255, cv2.NORM_MINMAX)
    # And swap Red and Blue channel as well
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
        swapRB=True, crop=False)
    net.setInput(blob)
    if W is None or H is None:
        (H, W) = image.shape[:2]
    layerOutputs = net.forward(ln)

    # initialize our lists of detected bounding boxes, confidences, and
    # class IDs, respectively
    boxes = []
    confidences = []
    classIDs = []
# loop over each of the layer outputs
    for output in layerOutputs:
        # loop over each of the detections
        for detection in output:
            # extract the class ID and confidence (i.e., probability) of
            # the current object detection
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]

            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > CONFIDENCE_THRESHOLD:
                # scale the bounding box coordinates back relative to the
                # size of the image, keeping in mind that YOLO actually
                # returns the center (x, y)-coordinates of the bounding
                # box followed by the boxes' width and height
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")

                # use the center (x, y)-coordinates to derive the top and
                # and left corner of the bounding box
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))

                # update our list of bounding box coordinates, confidences,
                # and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)

	# apply non-maxima suppression to suppress weak, overlapping boundingqq
	# boxes
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD,0.4)
    color=[0,255,0]
    # ensure at least one detection exists
    if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            
            cX=x+w//2
            cY=y+h//2
            cv2.circle(image, (cX,cY), 20, (255,0,0), -1)
            
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 10)
           
            text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
            cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,1, color, 2)
            #ser.write(b'L')
            if cY>=280 and cY<=330:
                if cX>=0 and cX<=213:
                    T_M=time.time()
                    state=T_M-P_MT
                    if state>3:
                        ser.write(b'M')
                        print('M')
                        P_MT=T_M
                elif cX>213 and cX<=426:
                    T_R=time.time()
                    state=T_R-P_RT
                    if state>3:
                        ser.write(b'R')
                        print('R')
                        P_RT=T_R
                elif cX>426 and cX<=640:
                    T_S=time.time()
                    state=T_S-P_ST
                    if state>3:
                        ser.write(b'S')
                        print('S')
                        P_ST=T_S
               
                
               
                

                
    # show the output image
    cv2.line(image, (0,300), (640,300), (255,0,0), 4)
    cv2.line(image, (213,0), (213,480), (255,0,0), 4)
    cv2.line(image, (426,0), (426,480), (255,0,0), 4)
    #fps_text="Fps is: {:.2f}".format(fps.fps())
    #cv2.putText(image, fps_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,1, color, 2)
    cv2.imshow("output_plant", cv2.resize(image,(800, 600)))
    writer.write(cv2.resize(image,(800, 600)))
    fps.update()
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    #elif key == ord("w"):
    #    brightness+=1
    #    vs.set(cv2.CAP_PROP_BRIGHTNESS,brightness)

fps.stop()

print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()

# release the file pointers
print("[INFO] cleaning up...")
writer.release()
vs.release()


# In[4]:


#cam = cv2.VideoCapture(0)

#       key value
#cam.set(3 , 640  ) # width        
#cam.set(4 , 480  ) # height       
#cam.set(10, 120  ) # brightness     min: 0   , max: 255 , increment:1  
#cam.set(11, 50   ) # contrast       min: 0   , max: 255 , increment:1     
#cam.set(12, 70   ) # saturation     min: 0   , max: 255 , increment:1
#cam.set(13, 13   ) # hue         
#cam.set(14, 50   ) # gain           min: 0   , max: 127 , increment:1
#cam.set(15, -3   ) # exposure       min: -7  , max: -1  , increment:1
#cam.set(17, 5000 ) # white_balance  min: 4000, max: 7000, increment:1
#cam.set(28, 0    ) # focus          min: 0   , max: 255 , increment:5


# In[ ]:





