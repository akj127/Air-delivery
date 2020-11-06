#!/usr/bin/env python
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
import rospy
import matplotlib.pyplot as plt
import math
import sys, random, math
from math import sqrt,cos,sin,atan2
import os
import roslib
import numpy as np
import csv
import time
import tf

#Variable Initialise
height = 0
vx = 0
vy = 0
v = 0
curr_x = 0
curr_y = 0
drop_x = 28.677697 
drop_y = 77.249312
dst_x = 100
dst_y = 100
pitch = 0
first_waypt = [0,0]
pehlibar = True
g = 9.8

def getVelocity(measurement):
	global vx,vy,v
	x=measurement.twist.linear.x
	y=measurement.twist.linear.y
	vx=math.sqrt(x ** 2)
	vy=math.sqrt(y ** 2)
	v = math.sqrt((vx**2) + (vy**2))

def dist(a,b):
	return math.sqrt((a[1]-b[1])**2+(a[0]-b[0])**2)

def getGPS(measurement):
    global curr_x,curr_y,pehlibar,first_waypt,drop_x,drop_y,dst_y,dst_x
    x=measurement.latitude 
    y=measurement.longitude
    if pehlibar == True:
    	first_waypt = [x,y]
    	print "Origin: ",first_waypt
    	dst_x , dst_y = getXY_waypt(drop_x,drop_y,first_waypt[0],first_waypt[1],0,0)
    	print "Drop Point: ",[dst_x,dst_y]
    	pehlibar = False
    curr_x,curr_y = getXY_waypt(x,y,first_waypt[0],first_waypt[1],0,0)

def getAltitude(measurement):
	global height
	height = measurement.data
	if height < 0:
		height = -1*height

def getPitch(measurement):
	global pitch
	a = measurement.orientation.x
	b = measurement.orientation.y
	c = measurement.orientation.z
	d = measurement.orientation.w
	[r,p,y] = tf.transformations.euler_from_quaternion((a,b,c,d))
	pitch = p
	#print pitch

def letsdrop():
	global height, vx, vy, v, curr_x, curr_y, pitch, g, first_waypt, dst_x, dst_y
	pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1000)
	control = OverrideRCIn()
	#control.channels[4] = 1000
	drop_now = 0
	pos_old = [0,0]
	#v = [0,0]
	landing_threshold = 10
	while(drop_now == 0):
		if pitch < 0:
			vxx = v*cos(pitch)
			vyy = -1*v*sin(pitch)
			R = (vxx)*(sqrt( vyy*vyy + 2*g*height ) + (vyy))/g
		else:
			vxx = v*cos(pitch)
			vyy = v*sin(pitch)	
			R = (vxx)*(sqrt( vyy*vyy + 2*g*height ) - (vyy))/g

		if curr_x != pos_old[0] and curr_y != pos_old[1]:
			v = [curr_x - pos_old[0] , curr_y - pos_old[1]]
			vmag = sqrt((v[0]**2) + (v[1]**2))
			u = [v[0]/vmag , v[1]/vmag]
			predpt = [curr_x + R*u[0], curr_y + R*u[1]]
		else:
			predpt = [curr_x, curr_y]

		check = dist(predpt, [dst_x,dst_y])
		print "Distance ",check
		print "Pitch ",pitch
		print "Velocity ",v
		if check < landing_threshold:
			drop_now = 1

		if drop_now == 1:
			control.channels[4] = 2000
			pub.publish(control)
			print "DROPPED"
		time.sleep(0.1)
		pos_old[0] = curr_x
		pos_old[1] = curr_y




#######   HAVERSINE START
def getXY_waypt(lat,lon,lat_pre,lon_pre,pre_x,pre_y):
	latrad = to_rad(lat)
	lonrad = to_rad(lon)
	lat_pre = to_rad(lat_pre)
	lon_pre = to_rad(lon_pre)
	if lat==lat_pre:
		pre_x=0
		pre_y=0
	else:
		pre_x,pre_y=haversine_waypt(lat_pre,lon_pre,latrad,lonrad,pre_x,pre_y)
	return pre_x,pre_y
	

def to_rad(deg):
	rad=(deg*math.pi)/180.0
	return rad

def to_deg(rad):
	deg=(rad*180)/math.pi
	return deg

def haversine_waypt(lat1,lon1,lat2,lon2,pre_x,pre_y):
	r=6372.8;
	dlat = lat2-lat1
	dlon = lon2-lon1
	a = (math.sin(dlat/2))**2 + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon/2))**2
    	c = 2 * math.atan2(math.sqrt(a),math.sqrt(1-a))
	d = r * c * 1000 
	y = math.sin(dlon) * math.cos(lat2);
	x = -1 * (math.sin(lat1)*math.cos(lat2)*math.cos(dlon)-math.cos(lat1)*math.sin(lat2))
	brng = math.fmod(math.atan2(y, x)+2*math.pi,2*math.pi)
	pre_x = pre_x +(d * math.cos(brng))
	pre_y = pre_y +(d * math.sin(brng))
	return pre_x,pre_y
######    HAVERSINE END


def listener():
   	rospy.init_node('listener', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	#r = rosrate(0.05)
	# rospy.Subscriber("/mavros/global_position/global", NavSatFix, plot_gps_measurements)
	rospy.Subscriber("/mavros/global_position/global", NavSatFix, getGPS)
	rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, getVelocity)
	rospy.Subscriber("/mavros/imu/data", Imu, getPitch)
	rospy.Subscriber("/mavros/global_position/rel_alt", Float64, getAltitude)

	letsdrop()
	rospy.spin()

if __name__ == '__main__':
    listener()
