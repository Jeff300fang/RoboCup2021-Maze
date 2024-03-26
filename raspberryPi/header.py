import cv2
import numpy as np
import serial
import time

import theSerial
coms = theSerial.serialSetup()


theVd = cv2.VideoCapture(4) #left
theCap = cv2.VideoCapture(0) #right

theVd.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
theVd.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

theCap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
theCap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

#left

knnLeft = cv2.ml.KNearest_create()
featuresLeft = np.loadtxt("/home/pi/maze/knn/knnDataLeft.txt", dtype=np.float32)
labelsLeft = np.loadtxt("/home/pi/maze/knn/knnLabelsLeft.txt", dtype=np.float32).reshape((featuresLeft.shape[0], 1))
knnLeft.train(featuresLeft, cv2.ml.ROW_SAMPLE, labelsLeft)

#right

knnRight = cv2.ml.KNearest_create()
featuresRight = np.loadtxt("/home/pi/maze/knn/knnDataRight.txt", dtype=np.float32)
labelsRight = np.loadtxt("/home/pi/maze/knn/knnLabelsRight.txt", dtype=np.float32).reshape((featuresRight.shape[0], 1))
knnRight.train(featuresRight, cv2.ml.ROW_SAMPLE, labelsRight)
