# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 17:41:50 2019

@author: Frank Michael Posada
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Sep 10 15:07:21 2019

@author: luisf
"""

import serial, time
import numpy as np
import math as mt
import cv2
import urllib


#===========================Object Detection===================================
#==============================================================================

#This function just reads and imagefrom the url (we are using ipwebcam) and 
#transforms it into an opencv image file to process
def url_to_image(url):
	# download the image, convert it to a NumPy array, and then read
	# it into OpenCV format
	resp = urllib.request.urlopen(url)
	image = np.asarray(bytearray(resp.read()), dtype="uint8")
	image = cv2.imdecode(image, cv2.IMREAD_COLOR)
 
	# return the image
	return image

#this function takes the image and detects the biggest object and returns
#its coordenates which is the object we want to grab using the Robot
def detect_object(url):
    img = url_to_image(url)
    r,c,l = img.shape
    
    #Reducing the size of the image
    img_toshow = cv2.resize(img, (int(c*0.3),int(r*0.3)), cv2.INTER_AREA)
    
    #removing shadows. Optional#######
    img_g = cv2.cvtColor(img_toshow, cv2.COLOR_BGR2GRAY)
    img_d = cv2.dilate(img_g, np.ones((15,15), np.uint8), iterations = 1)
    img_b = cv2.medianBlur(img_d, 41)
    diff_img = 255 - cv2.absdiff(img_g, img_b)
    norm_img = diff_img.copy() # Needed for 3.x compatibility
    cv2.normalize(diff_img, norm_img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    _, thr_img = cv2.threshold(norm_img, 150, 255, cv2.THRESH_BINARY_INV)
    #############
    
    
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
    ppcx = 17.06 #pixels per centimeter for our setup
    ppcy = 12.4
    xpos = np.around(xpos/ppcx,2)
    ypos = np.around(ypos/ppcy,2) +4.4 - 3
    return xpos,ypos
#==============================================================================

#==============================================================================
#=============================Direct Kinematics================================
def dkine(q1,q2,q3,q4,l1,l2,l3,l4):
    q3 = -q3
    q4 = -q4
    #Homogeneus Transformation Matrices
    T01 =np.array([[ mt.cos(q1) ,0 ,mt.sin(q1) ,0 ],
                    [ mt.sin(q1) ,0 ,-mt.cos(q1) ,0 ],
                    [ 0 ,1 ,0 ,l1 ],
                    [ 0 ,0 ,0 ,1 ]])  #ecuation for joint 1 seen from 0
    T12 =np.array([[ mt.cos(q2) ,-mt.sin(q2) ,0 ,l2*mt.cos(q2) ],
                    [ mt.sin(q2) ,mt.cos(q2) ,0 ,l2*mt.sin(q2) ],
                     [0 ,0 ,1 ,0 ],
                    [0 ,0 ,0 ,1 ]]) #ecuation for point 2 seen from 1
    T23 =np.array([[ mt.cos(q3) ,-mt.sin(q3) ,0 ,l3*mt.cos(q3) ],
                    [ mt.sin(q3) ,mt.cos(q3) ,0 ,l3*mt.sin(q3) ],
                    [ 0, 0 ,1 ,0], 
                    [ 0, 0 ,0 ,1]]) #ecuation for point 3 seen from 2
    T34 =np.array([[ mt.cos(q4) ,-mt.sin(q4) ,0 ,l4*mt.cos(q4) ],
                    [ mt.sin(q4) ,mt.cos(q4) ,0 ,l4*mt.sin(q4) ],
                    [ 0 ,0 ,1 ,0 ],
                    [ 0 ,0 ,0 ,1 ]]) #ecuation for point 4 seen from 3
    #now we multiply them to obtain T04 (they are matrices, so the order matters a lot)
    #T04= T01*T12*T23*T34
    T04=np.dot( T01 , np.dot( T12 , np.dot( T23, T34) ) )
    return T04

#==============================================================================

#==============================================================================
#                Inverse Kinematic equations for q1,q2,q3 and q4
#==============================================================================
def car2joint(x,y,z,phi,l1,l2,l3,l4):
    q1 = mt.atan2(y,x)
    
    cp = mt.sqrt( x**2 + y**2 + ( z-l1 )**2 )
    
    cq = mt.sqrt( ( x-l4*mt.cos(phi)*mt.cos(q1) )**2 +
                 ( y-l4*mt.cos(phi)*mt.sin(q1) )**2 +
                 ( z-l1+l4*mt.sin(phi) )**2 )
    #print(cq)
    
    #alp = mt.acos( (l2**2 + cq**2 - l3**2) /
    #              (2*l2*cq) )              
    #bet = mt.acos( (cq**2 + cp**2 - l4**2) /
    #              (2*cq*cp) )           acording to some sources its better to use
    #                                    atan2 instead of acos for accuracy.
    #                                    using a trigonometric identity we can 
    #                                    transform cos into tan and vice versa 
    
    alp = 2*mt.atan2( mt.sqrt( 2*l2*cq - l2**2 - cq**2 + l3**2) ,
                     mt.sqrt( 2*l2*cq + l2**2 + cq**2 - l3**2 ) )
    
    bet = 2*mt.atan2( mt.sqrt( 2*cq*cp - cq**2 - cp**2 + l4**2) ,
                     mt.sqrt( 2*cq*cp + cq**2 + cp**2 - l4**2 ) ) 
    
    gam = mt.atan2((z-l1),mt.sqrt(x**2 + y**2))
    
    q2 = alp + bet + gam
    q3 = mt.pi - mt.acos( (l2**2 + l3**2 - cq**2) / (2*l2*l3) )
    q4 = phi + q2 -q3
    return q1,q2,q3,q4

def ikine(x,y,z,l1,l2,l3,l4, asdegrees = True):
    phian = 0
    while True:
        phian = phian + 1
        if( phian > 180 ):
            print("=========Singularidad=========")
            break
        try:
            phi = ( phian * mt.pi) / 180
            q1,q2,q3,q4 = car2joint(x,y,z,phi,l1,l2,l3,l4)
            if(  not( (q1 < 0) or (q2 < 0) or (q3 < 0) or (q4 < 0) ) ):
                break
        except:
            pass
    if(asdegrees == True):
        q1 = round((q1 * 180) / mt.pi)
        q2 = round((q2 * 180) / mt.pi)
        q3 = round((q3 * 180) / mt.pi)
        q4 = round((q4 * 180) / mt.pi)
        return q1,q2,q3,q4
    else:
        return q1,q2,q3,q4
#==============================================================================

#==========================Function to move the Robot==========================
def moveto(x,y,z,g,l1,l2,l3,l4):

    #we get the angles for the joints using the IK function from above
    q1,q2,q3,q4 = ikine(x,y,z,l1,l2,l3,l4)
    
    #sending the move orders through serial to the arduino
    for i in range(2):
        #send information through port
        micro.write(  bytearray('D'+str(q1), 'utf-8' ) )
        #receives information through port
        
        time.sleep(0.001)
        micro.write(  bytearray('D'+str(q2), 'utf-8' ) )
        #receives information through port
        
        time.sleep(0.001)
        micro.write(  bytearray('D'+str(q3), 'utf-8' ) )
        #receives information through port
        
        time.sleep(0.001)
        micro.write(  bytearray('D'+str(q4), 'utf-8' ) )
        #receives information through port
        
        time.sleep(0.001)
        micro.write(  bytearray('D'+str(g), 'utf-8' ) )
        #receives information through port
        time.sleep(0.5)
#==============================================================================

#==============================================================================
#=                                Program Start                               =
#==============================================================================
#open serial communication with port COM9
micro = serial.Serial(#serial connection
    port='COM3',
    baudrate=9600)

time.sleep(2)


#%%
#================================Robot dimensions==============================
l1 = 5.5
l2 = 8.3
l3 = 8.2
l4 = 9.5

#url of the image taken from the phone using the app (ipwebcam)
url = 'http://10.1.129.199:8080/photo.jpg'

#gets the position of the object
xpos,ypos = detect_object(url)
print(xpos,ypos)

#%%
#===========================Sequence of movements==============================
moveto(xpos,ypos, 3 ,45,l1,l2,l3,l4)
time.sleep(5)
moveto(xpos,ypos, 3 ,90,l1,l2,l3,l4)
time.sleep(5)
moveto(-17,17,3,45,l1,l2,l3,l4)
time.sleep(5)
moveto(0,10,7,90,l1,l2,l3,l4)


#%% ==============================Close Communication==========================
#close serial communication
micro.close()



