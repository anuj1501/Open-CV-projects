#import libraries
import cv2
import time
import numpy as np


#Read the web cam
cap = cv2.VideoCapture(0)

#Allow the system to sleep for 3 seconds before the webcam starts
time.sleep(3)
count = 0
background = 0

#capture the background in range of 60
for i in range(60):

    ret,background = cap.read()

background = np.flip(background, axis = 1)

##Read every frame from the webcam , until the camera is open
while True:

    ret,img = cap.read()

    if not ret:

        break

    img = np.flip(img, axis = 1)

    ##convert the color space from BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ##generate masksto detect red color
    lower_red = np.array([0,120,50])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(hsv,lower_red,upper_red)

    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(hsv,lower_red,upper_red)

    mask1 = mask1 + mask2

    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_DILATE, np.ones((3,3),np.uint8))

    ##create an inverted mask to segment out the red color from the frame
    mask2 = cv2.bitwise_not(mask1)

    #segment the red color part out of the frame using bitwise and with the inverted mask
    res1 = cv2.bitwise_and(img,img,mask=mask2)

    ##create image showing static background frame pixels only for masked region
    res2 = cv2.bitwise_and(background,background, mask=mask1)


    ##generating the final output
    finalOutput = cv2.addWeighted(res1,1,res2,1,0)
    cv2.imshow("magic",finalOutput)
    cv2.imshow("magic1",res1)
    cv2.imshow("magic2",res2)

    count += 1
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()









