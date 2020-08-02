#!/usr/bin/env python
import numpy as np
import cv2 as cv

def test1(im):
    imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(imgray, 127, 255, 0)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE) # cv.CHAIN_APPROX_SIMPLE)
    #cnt = contours[4]
    #cv.drawContours(imgray, [cnt], 0, (0,255,0), 3)
    cv.drawContours(imgray, contours, -1, (0,255,0), 3)
    cv.imshow('Contours', imgray)
    cv.waitKey(0)

    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    #cnt = contours[4]
    #cv.drawContours(imgray, [cnt], 0, (0,255,0), 3)
    cv.drawContours(imgray, contours, -1, (0,255,0), 3)
    cv.imshow('Contours', imgray)

    cv.waitKey(0)

    cv.destroyAllWindows()
def test2(im):
    im = cv.cvtColor(im, cv.COLOR_BGR2GRAY)

    ret, thresh = cv.threshold(im, 127, 255, 0)
    contours, hierarchy = cv.findContours(thresh, 1, 2)
    cnt = contours[0]
    M = cv.moments(cnt)
    print(M)

im = cv.imread('C:\\Users\\wangx\\SkyStone\\opencv_py\\skystones_2.jpg')
def nothing(x):
    pass

useCamera = False # True

# Create a window
cv.namedWindow('image')

# create trackbars for color change
cv.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
cv.createTrackbar('SMin','image',0,255,nothing)
cv.createTrackbar('VMin','image',0,255,nothing)
cv.createTrackbar('HMax','image',0,179,nothing)
cv.createTrackbar('SMax','image',0,255,nothing)
cv.createTrackbar('VMax','image',0,255,nothing)

# Set default value for MAX HSV trackbars.
cv.setTrackbarPos('HMax', 'image', 179)
cv.setTrackbarPos('SMax', 'image', 255)
cv.setTrackbarPos('VMax', 'image', 255)

# Initialize to check if HSV min/max value changes
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

# Output Image to display
if useCamera:
    cap = cv.VideoCapture(0)
    # Wait longer to prevent freeze for videos.
    waitTime = 330
else:
    img = cv.imread('C:\\Users\\wangx\\SkyStone\\opencv_py\\skystones_2.jpg')
    output = img
    waitTime = 33

while(1):

    if useCamera:
        # Capture frame-by-frame
        ret, img = cap.read()
        output = img

    # get current positions of all trackbars
    hMin = cv.getTrackbarPos('HMin','image')
    sMin = cv.getTrackbarPos('SMin','image')
    vMin = cv.getTrackbarPos('VMin','image')

    hMax = cv.getTrackbarPos('HMax','image')
    sMax = cv.getTrackbarPos('SMax','image')
    vMax = cv.getTrackbarPos('VMax','image')

    # Set minimum and max HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Create HSV Image and threshold into a range.
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    output = cv.bitwise_and(img,img, mask= mask)

    # Print if there is a change in HSV value
    if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
        print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
        phMin = hMin
        psMin = sMin
        pvMin = vMin
        phMax = hMax
        psMax = sMax
        pvMax = vMax

    # Display output image
    cv.imshow('image',output)

    # Wait longer to prevent freeze for videos.
    if cv.waitKey(waitTime) & 0xFF == ord('q'):
        break

# Release resources
if useCamera:
    cap.release()
cv.destroyAllWindows()