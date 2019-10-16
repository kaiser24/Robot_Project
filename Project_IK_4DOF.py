# -*- coding: utf-8 -*-
"""
Created on Wed Aug 14 15:16:57 2019

@author: luisf
"""
import numpy as np
import math as mt

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

#testing
#================================Robot dimensions==============================
l1 = 5.6
l2 = 8.3
l3 = 8.2
l4 = 9.5

#================================input variables===============================
#==================FOR TESTING. CHANGE THE CARTESIAN VARIABLES=================
#==============================================================================


x = 0
y = 15
z = 6

#==============================================================================
#=========================Finding the angles with the IK=======================

q1,q2,q3,q4 = ikine(x,y,z,l1,l2,l3,l4, asdegrees = True)
    
#==============================================================================
print("  ")
print("Cinematica inversa: ")
print("q1 : ",q1," q2: ",q2," q3: ",q3," q4: ",q4)
print("  ")


#from degree to radians to calculate the DK to confirm the position
q1 = (q1 * mt.pi) / 180
q2 = (q2 * mt.pi) / 180
q3 = (q3 * mt.pi) / 180
q4 = (q4 * mt.pi) / 180

T04 = dkine(q1,q2,q3,q4,l1,l2,l3,l4)


print("Los anteriores Angulos llevan a la siguiente posición: ")
print("x: ",np.around(T04[0,3],3),"y: ",np.around(T04[1,3],3) ,"z: ", np.around(T04[2,3] , 3) )