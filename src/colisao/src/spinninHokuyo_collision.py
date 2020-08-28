#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud,LaserScan
from array import array
from math import atan2,degrees,acos,sqrt,cos,sin
from std_msgs .msg import Int32,String,Int16
import numpy as np
from geometry_msgs.msg import Vector3
from copy import copy

laser_visual = LaserScan()

def callback(data):
    collision_flag = 0
    min_value  = 2.00
    ScanVec = np.array([])
    angle_object = np.array([])
    L_min = 0
    a_increment = 0
    angle = 0
    x=0
    y=0
    rj_x = ([])
    rj_y = ([])
    threshold = 0.05
    dif_x = 0
    dif_y = 0
    old_rjx = 0
    old_rjy = 0
    count = 0
    list_angle = []
    global laser_visual
    laser_visual = copy(data)
    laser_visual.ranges = []
    for i in range(0,len(data.ranges)):
        ScanVec = np.append(ScanVec , float(data.ranges[i])) #Salva dados das distancias do laser
        L_min = np.degrees(data.angle_min) # salva angulo minimo do laser
        a_increment =  np.degrees(data.angle_increment) # salva resolucao angular do laser



        if(ScanVec[i] !=0 and ScanVec[i]<min_value):
            collision_flag=1            
        if (collision_flag==1):            
            angle = L_min + a_increment*i # calcula o angulo do ponto 
            list_angle.append(angle)
            x = ScanVec[i]*cos(angle) # calcula a coordenada x do ponto
            rj_x =  np.append(rj_x, x) 
            y = ScanVec[i]*sin(angle) # calcula coordenada y do ponto
            rj_y = np.append(rj_y, y)
            collision_flag = 0
        i=i+1
        if (i ==len(data.ranges)):            
            for k in range (0, len(rj_x)): 
                if(count ==1):
                    old_rjx = rj_x[k]
                    old_rjy = rj_y[k]                    
                if(count>1):
                    dif_x = rj_x[k] - old_rjx # calcula a diferenca do ponto x anterior com o atual
                    dif_y = rj_y[k] - old_rjy # calcula a diferenca do ponto y anteriro com o atual
                    old_rjx = rj_x[k]
                    old_rjy = rj_y[k]
                    if(abs(dif_x) > threshold and abs(dif_y)>threshold): # caso a diferenca seja grande detecta aqueles pontos como descontinuidade
                        laser_visual.ranges.append(data.ranges[k])
			print "x ", rj_x[k]," y ",rj_y[k]
                        #print list_angle[k], "\n"
                    else:
			print "e"
                        laser_visual.ranges = []
                count = count+1
                k = k+1
            count = 0
            dif_x = 0
            dif_y = 0
            old_rjx = 0
            old_rjy = 0
            j = 0
            i = 0
            led = 0
            ScanVec = []
            rj_y = []
            rj_x = []


def getdata():
    rospy.init_node("detect_colision",anonymous=True)
    rospy.Subscriber("/scan",LaserScan,callback)
    visual = rospy.Publisher("Pontos_colisao",LaserScan,queue_size=10)
    rate = rospy.Rate(100   )
    while not rospy.is_shutdown():
        visual.publish(laser_visual)
        rate.sleep()
if __name__== '__main__':
	try:
	    getdata()
	except rospy.ROSInterruptException:
		pass

