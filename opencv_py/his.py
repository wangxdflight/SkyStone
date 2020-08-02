import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
print( cv.__version__ )

#---- Gray image
histsize = 256 #Because we are working on grayscale pictures which values within 0-255

flags = [i for i in dir(cv) if i.startswith('COLOR_')]
print( flags )

orig = cv.imread("test.png", cv.IMREAD_COLOR)
height, width = img.shape[:2]
print(height, width)

orig = cv.resize(orig, (int(height/2), int(width/2)))                    # Resize image
cv.imshow('original', orig)
cv.waitKey(0)

plt.figure()
img = cv.cvtColor(orig, cv.COLOR_BGR2GRAY)
cv.imshow('GRAY', img)

plt.figure()
img = cv.cvtColor(orig, cv.COLOR_BGR2HSV)
cv.imshow('HSV', img)

cv.waitKey(0)
hist = cv.calcHist([img],[0],None,[256],[0,256])
hist,bins = np.histogram(img.ravel(),256,[0,256])
plt.hist(img.ravel(),256,[0,256]); plt.show()

# color = ('b','g','r')
# for i,col in enumerate(color):
#     histr = cv.calcHist([img],[i],None,[256],[0,256])
#     plt.plot(histr,color = col)
#     plt.xlim([0,256])
# plt.show()
