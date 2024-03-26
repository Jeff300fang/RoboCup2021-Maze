import numpy as np
import cv2
import header

black_low = np.asarray([0,0,35])
black_high = np.asarray([179,254,254])

def getLetter():
        ret, frame = header.theVd.read()
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(gray, black_low, black_high)
        mask = np.bitwise_not(mask)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if contours is not None:
                for contour in contours:
                        x,y,w,h = cv2.boundingRect(contour)
                        if(w*h < 100):
                                continue
                        if(not(0.9 <= float(h)/w <= 1.5)):
                                continue 
                        if(not(2<= (w*h)/cv2.contourArea(contour) <= 5) ):
                                continue
                        new = mask[y:y+h,x:x+w]
                        resized = cv2.resize(new,(30,30))

                        one = resized.reshape(1,900).astype(np.float32)
                        ret, res, neighbours, distance = header.knnLeft.findNearest(one, 9) 
                        if(distance[0][0] > 15000000):
                                return(0)
                        else:
                                theLetter = chr(res[0][0])
                                if(theLetter == 'h'):
                                        return(1)
                                if(theLetter == 's'):
                                        return(2)
                                if(theLetter == 'u'):
                                        return(3)

