import cv2
import numpy as np
import time

def get_contours(img):

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(img_hsv,lower_red,upper_red)

    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(img_hsv,lower_red,upper_red)

    mask = mask1 + mask2

    contour, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    x,y,w,h = 0,0,0,0

    for ctr in contour:

        peri = cv2.arcLength(ctr, True)

        approx = cv2.approxPolyDP(ctr, 0.01*peri, True)

        if cv2.contourArea(ctr) > 200:

            x,y,w,h = cv2.boundingRect(approx)

    return mask,x+w,y

def get_point(img):

    allpoints = []

    _,x,y = get_contours(img)

    cv2.circle(img, (x,y),5,(0,165,255), cv2.FILLED)

    cv2.imshow("output3",img)

    if x!=0 and y!=0:
        allpoints.append([x,y])

    return allpoints

def drawoncam(img, points):

    for pts in points:

        cv2.circle(img,(pts[0],pts[1]),5,(0,165,255), cv2.FILLED)

all_points = []
#Read the web cam
cap = cv2.VideoCapture(0)

#Allow the system to sleep for 3 seconds before the webcam starts
time.sleep(5)

while True:

    ret,img = cap.read()

    img_flip = cv2.flip(img,1)

    mask,_,_ = get_contours(img_flip)

    points = get_point(img_flip)

    if len(points) != 0:

        for pts in points:

            all_points.append(pts)
    
    if len(all_points) != 0:

        drawoncam(img_flip,all_points)

    cv2.imshow("output1",img_flip)

    cv2.imshow("output2",mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        
        break

cap.release()
cv2.destroyAllWindows()