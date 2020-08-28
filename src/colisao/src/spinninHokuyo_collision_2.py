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

global laser_visual
laser_visual = LaserScan()
#global j,ScanVec,angle_object,L_min,a_increment


class Collision:
	def __init__(self):
		self.j = 0        	
		self.scanvec = np.array([])
        	self.angle_object = np.array([])
        	self.l_min = 0
        	self.a_increment = 0
		self.j = 0

        def init_node(self):	

        	rospy.init_node("detect_colision",anonymous=True)
        	rospy.Subscriber("/scan",LaserScan,callback)
		
		rospy.loginfo("Watting for /scan...")
		try:
			rospy.wait_for_message('/scan',LaserScan,timeout=0.5)
			rospy.loginfo("/scan received")
		except Exception as e:
			rospy.logger("/scan msg not found...")
		self.get_time()
        	
	def get_time(self):
		now = rospy.Time.now()
		self.time = now.to_sec()

	

	def scan_callback(self,msg):
		self.laser_visual = msg
		self.scanvec = np.append(self.ScanVec, msg.ranges)
		self.l_min = np.degrees(msg.angle_min)
		self.a_increment = np.degrees(msg.angle_increment)
		self.j = len(msg.ranges)
	
		for i in range(0,j):
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
				                 rospy.loginfo("colisao")
                    #print list_angle[k], "\n"
				                else:

					        	count = count+1
					                k = k+1
			cont = 0
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


	if __name__== '__main__':	

		rospy.loginfo("scan collision data")
		scan_collision = Collision()
		rate = rospy.Rate(10)
	
		while not rospy.is_shutdown():
			try:




				
				
    
            
 
