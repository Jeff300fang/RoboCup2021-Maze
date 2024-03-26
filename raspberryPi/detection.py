import cv2
import numpy as np
import header
import knnFinalL
import knnFinalR
import time

import theSerial


#left

vd = header.theVd

#right

cap = header.theCap

#black right

black_low_right = np.asarray([0,0,53])
black_high_right = np.asarray([179,254,254])

#black left

black_low_left = np.asarray([0,0,35])
black_high_left = np.asarray([179,254,254])

#general color left
color_low_left = np.asarray([0,91,0])
color_high_left = np.asarray([179,254,254])

#general color right
color_low_right = np.asarray([0,44,0])
color_high_right = np.asarray([179,254,254])

#left green

left_green_low = np.asarray([56,31,0])
left_green_high = np.asarray([108,254,254])

#left yellow

left_yellow_low = np.asarray([16,131,0])
left_yellow_high = np.asarray([31,254,254])

#left red
left_red_low = np.asarray([67,132,0])
left_red_high = np.asarray([179,254,254])


#right green

right_green_low = np.asarray([68,62,0])
right_green_high = np.asarray([87,255,255])

#left yellow

left_yellow_low = np.asarray([16,131,0])
left_yellow_high = np.asarray([31,254,254])
#left red
left_red_low = np.asarray([67,132,0])
left_red_high = np.asarray([179,254,254])


def checkRight():
        ret,frame = cap.read()
       resized = cv2.resize(frame,(160,120))
        hsv = cv2.cvtColor(resized.copy(), cv2.COLOR_BGR2HSV)
        gray = cv2.inRange(hsv.copy(),black_low_right,black_high_right)
        flipped = cv2.bitwise_not(gray)
        (contours, h) = cv2.findContours(flipped.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        flag = 0
        if(len(contours) > 0):
                for cont in contours:
                        x,y,w,h = cv2.boundingRect(cont)
                        if(w < 25 or h < 25):
                                continue
                        return(1)
                        cv2.rectangle(resized,(x,y),(x+w,y+h),(0,255,0),2)
        colored = cv2.inRange(hsv,color_low_right,color_high_right)
        (ccContours,cch) = cv2.findContours(colored, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(ccContours) > 0:
                for ccCont in ccContours:

                        ccX,ccY,ccW,ccH = cv2.boundingRect(ccCont)
                        #cv2.rectangle(resized,(ccX,ccY),(ccX+ccW,ccY+ccH),(0,255,0),2)
                        theSum = np.sum(resized[ccY:ccY+ccH,ccX:ccX+ccW])
                        #print(theSum)
                        #print(ccW,ccH)

                        if(ccW < 50 or ccH < 50):
                                continue
                        if(theSum < 1000000):
                               continue
                        return(2)
        cv2.imshow("gray",resized)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
                return(0)
        return(-1)


def checkLeft():
        ret,frame = vd.read()
        resized = cv2.resize(frame,(160,120))
        hsv = cv2.cvtColor(resized.copy(), cv2.COLOR_BGR2HSV)
        colored = cv2.inRange(hsv.copy(), color_low_left, color_high_left)
        (ccContours, ccH) = cv2.findContours(colored.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(ccContours) > 0:
                for ccCont in ccContours:
                        ccX,ccY,ccW,ccH = cv2.boundingRect(ccCont)
                        if((ccW < 25 and ccH < 25) or (ccW > 60 and ccH > 60)):
                                continue
                        theSum = np.sum(colored[ccY:ccY+ccH,ccX:ccX+ccW])
                        if(theSum < 250000):
                                continue
                        #print(theSum)
                        return(2)

        gray = cv2.inRange(hsv.copy(),black_low_left,black_high_left)
        flipped = cv2.bitwise_not(gray)
        (contours, h) = cv2.findContours(flipped.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        flag = 0
        if len(contours) > 0:
                for cont in contours:
                        x,y,w,h = cv2.boundingRect(cont)
                        if(w < 25 or h < 25):
                                continue
                        if w/h > 1:
                                continue
                        return(1)
                        cv2.rectangle(resized,(x,y),(x+w,y+h),(0,255,0),2)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
                return(0)
        return(-1)


def getLetterLeft():
        occArr = [0,0,0,0]
        for i in range(0,25):
                ret = knnFinalL.getLetter()
                if(ret != 0 and ret is not None):
                        occArr[ret] +=1
        max = occArr[1]
        sol = 1
        print(occArr)
        for i in range(2,4):
                if(occArr[i] > max):
                        max = occArr[i]
                        sol = i
        return(sol)
        if sol == 3:
                info = bytes([0])
        if sol == 2:
                info = bytes([1])
        if sol == 1:
                info = bytes([3])
        if sol == 0:
                info = bytes([0])
        #header.coms.write(info)
def getLetterRight():
        occArr = [0,0,0,0]
        for i in range(0,25):
                ret = knnFinalR.getLetter()
                if(ret != 0 and ret is not None):
                       occArr[ret] +=1
        max = occArr[1]
        sol = 1
        print(occArr)
        for i in range(2,4):
                if(occArr[i] > max):
                        max = occArr[i]
                        sol = i
        return(sol)

def getColorLeft():
        ret,frame = vd.read()
        hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)
        gray = cv2.inRange(hsv.copy(),color_low_left,color_high_left)
        (ccContours, ccH) = cv2.findContours(gray.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(ccContours) > 0:
                for ccCont in ccContours:
                        ccX,ccY,ccW,ccH = cv2.boundingRect(ccCont)
                        if(ccW < 130 and ccH < 130):
                                continue

                        theSum = np.sum(gray[ccY:ccY+ccH,ccX:ccX+ccW])
                        if(theSum < 10000):
                                continue
                        if(theSum/ccW/ccH/255 < 0.6):
                                continue
                        resized = cv2.resize(hsv.copy(),(160,120))
                        frame = cv2.resize(frame,(160,120))
                        #check green
                        grayGreen = cv2.inRange(resized.copy(),left_green_low,left_green_high)
                        (gContours,gH) = cv2.findContours(grayGreen.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if len(gContours) > 0:
                                for gCont in gContours:
                                        gX,gY,gW,gH = cv2.boundingRect(gCont)
                                        if gW < 20 and gH < 20:
                                                continue
                                        cv2.rectangle(frame,(gX,gY),(gX+gW,gY+gH),(0,255,0),2)
                                        cv2.imshow("frame",frame)
                                        key = cv2.waitKey(1) & 0xFF
                                        return(3)
                        #check yellow
                        grayYellow = cv2.inRange(resized.copy(),left_yellow_low,left_yellow_high)
                        (yContours,yH) = cv2.findContours(grayYellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.imshow("frame",grayYellow)
                        cv2.waitKey(1)

                        if len(yContours) > 0:
                                for yCont in yContours:
                                        yX,yY,yW,yH = cv2.boundingRect(yCont)
                                        if yW < 20 and yH < 20:
                                                continue
                                        cv2.rectangle(frame,(yX,yY),(yX+yW,yY+yH),(0,255,0),2)
                                        cv2.imshow("frame",grayYellow)
                                        key = cv2.waitKey(1) & 0xFF
                                        return(2)

                        #check red
                        grayRed = cv2.inRange(resized.copy(),left_red_low,left_red_high)
                        (rContours,rH) = cv2.findContours(grayRed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                        if len(rContours) > 0:
                                for rCont in rContours:
                                        rX,rY,rW,rH = cv2.boundingRect(rCont)
                                        if rW < 20 and rH < 20:
                                                continue
                                        cv2.rectangle(frame,(rX,rY),(rX+rW,rY+rH),(0,255,0),2)
                                        cv2.imshow("frame",frame)
                                        key = cv2.waitKey(1) & 0xFF
                                       return(1)


        #cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
                return(0)

def getColorRight():
        ret,frame = cap.read()
        hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)
        gray = cv2.inRange(hsv.copy(),color_low_right,color_high_right)
        (ccContours, ccH) = cv2.findContours(gray.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(ccContours) > 0:
                for ccCont in ccContours:
                        ccX,ccY,ccW,ccH = cv2.boundingRect(ccCont)
                        if(ccW < 130 and ccH < 130):
                                continue

                        theSum = np.sum(gray[ccY:ccY+ccH,ccX:ccX+ccW])
                        if(theSum < 15000000):
                                continue
                        if(theSum/ccW/ccH/255 < 0.6):
                                continue
                        cv2.rectangle(frame,(ccX,ccY),(ccX+ccW,ccY+ccH),(0,255,0),2)
                        resized = cv2.resize(hsv.copy(),(160,120))
                        frame = cv2.resize(frame,(160,120))
                        #check green

                        grayGreen = cv2.inRange(resized.copy(),right_green_low,right_green_high)
                        (gContours,gH) = cv2.findContours(grayGreen.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if len(gContours) > 0:
                                for gCont in gContours:
                                        gX,gY,gW,gH = cv2.boundingRect(gCont)
                                        if gW < 40 and gH < 40:
                                                continue
                                        cv2.rectangle(frame,(gX,gY),(gX+gW,gY+gH),(0,255,0),2)
                                        cv2.imshow("frame",frame)
                                        key = cv2.waitKey(1) & 0xFF
                                        return(3)
        cv2.imshow("frame",frame)
        key = cv2.waitKey(1) & 0xFF

def getTheColorLeft():
        occArr = [0,0,0,0]
        for i in range(0,25):
                ret = getColorLeft()
                if(ret != 0 and ret is not None):
                        occArr[ret] +=1
        max = occArr[1]
        sol = 1
        print(occArr)
        for i in range(2,4):
                if(occArr[i] > max):
                        max = occArr[i]
                        sol = i
        if sol == 1:
                info = bytes([1])
        if sol == 2: 
                info = bytes([1])
        if sol == 3:
               info = bytes([0])
        if sol == 0:
                info = bytes([0])
        #header.coms.write(info)

while(vd.isOpened() and cap.isOpened()):
        comsRet = theSerial.pinged()
        if(comsRet == b'1'):
                retLeft = checkLeft()
                if(retLeft == 0):
                        break
                if retLeft == 1:
                        print("Letter Left")
                        info = bytes([ord('B')])
                        header.coms.write(info)
                        time.sleep(2)
                        result = getLetterLeft()
                        if(result == 1):
                              info = bytes([ord('H')])
                        elif(result == 2):
                              info = bytes([ord('S')])
                        else:
                              info = bytes([ord('U')])
                        header.coms.write(info)
                        #header.coms.write(getLetterLeft())
                #if retLeft == 2:
                     #   print("Color")
                        #info = bytes([8])
                        #header.coms.write(info)
                        #header.coms.write(getTheColorLeft())
                if retLeft == -1:
                        info = bytes([ord('C')])
                        header.coms.write(info)
        if(comsRet == b'3'):
                retRight = checkRight()
                if retRight == 1:
                        print("Letter Right")
                        info = bytes([ord('B')])
                        header.coms.write(info)
                        time.sleep(2)
                        result = getLetterRight()
                        print("Result:",result)
                        if(result == 1):
                              info = bytes([ord('H')])
                        elif(result == 2):
                              info = bytes([ord('S')])
                        else:
                              info = bytes([ord('U')])
                        header.coms.write(info)
                        #header.coms.write(getLetterLeft())
                #if retLeft == 2:
                     #   print("Color")
                        #info = bytes([8])
                        #header.coms.write(info)
                        #header.coms.write(getTheColorLeft())
                if retRight == -1:
                        info = bytes([ord('C')])
                        header.coms.write(info)


vd.release()
cv2.destroyAllWindows()






