# -*- coding: utf-8 -*-
"""
Created on Thu Oct 10 17:52:17 2019

@author: luisf
"""
#2
import numpy as np
import urllib
import cv2


def url_to_image(url):
	# download the image, convert it to a NumPy array, and then read
	# it into OpenCV format
	resp = urllib.request.urlopen(url)
	image = np.asarray(bytearray(resp.read()), dtype="uint8")
	image = cv2.imdecode(image, cv2.IMREAD_COLOR)
 
	# return the image
	return image

url = 'http://10.1.129.199:8080/photo.jpg'
img = url_to_image(url)
r,c,l = img.shape

#Reducing the size of the image
img_toshow = cv2.resize(img, (int(c*0.3),int(r*0.3)), cv2.INTER_AREA)

#removing shadows.
img_g = cv2.cvtColor(img_toshow, cv2.COLOR_BGR2GRAY)
img_d = cv2.dilate(img_g, np.ones((15,15), np.uint8), iterations = 1)
img_b = cv2.medianBlur(img_d, 41)
diff_img = 255 - cv2.absdiff(img_g, img_b)
norm_img = diff_img.copy() # Needed for 3.x compatibility
cv2.normalize(diff_img, norm_img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
_, thr_img = cv2.threshold(norm_img, 150, 255, cv2.THRESH_BINARY_INV)

#cv2.normalize(thr_img, thr_img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

#cheap method for filling holes
thr_img = cv2.dilate(thr_img, np.ones((12,12), np.uint8), iterations = 2)
thr_img = cv2.erode(thr_img, np.ones((12,12), np.uint8), iterations = 2)

thr_img[300:thr_img.shape[0],:] = 0

#finding contours
_, contours,_ = cv2.findContours(thr_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


#img_toshow = cv2.merge([img_g,img_g,img_g])
img_toshow = cv2.merge([img_g,img_g,img_g])

#finding the biggest object on the image
ux,uy,uw,uh = 0,0,0,0
uarea = 0
#cv2.drawContours(img_toshow, contours,-1, (0,0,255), 3)
for contour in contours:
    x,y,w,h = cv2.boundingRect(contour)
    area = w*h
    if(area > uarea):
        uarea = area
        ux,uy,uw,uh = x,y,w,h

#remarking the biggest object
cv2.rectangle(img_toshow, (ux,uy), (ux+uw,uy+uh), (0,0,255), 3)

#coordenates of the center of the object 

xpos, ypos = ux + (w/2), uy + (h/2)
rn,cn = thr_img.shape
xpos = xpos - (cn/2)
ypos = (rn - ypos)

#pixel coordenates to centimeters
ppcx = 17.06 #oixels per centimeter
ppcy = 12.4
xpos = np.around(xpos/ppcx,2)
ypos = np.around(ypos/ppcy,2) +4.4 - 3

print(xpos,ypos)


cv2.imshow('mask',thr_img)
cv2.imshow('foto',img_toshow)

cv2.waitKey(0)
cv2.destroyAllWindows()