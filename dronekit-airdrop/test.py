#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import matplotlib.pyplot as plt
import math
import sys, random, math
from math import sqrt,cos,sin,atan2
import os
import numpy as np
import csv
import time
#import tf
from dronekit import connect, Command, VehicleMode,LocationGlobal, LocationGlobalRelative

#Variable Initialise
height = 0
vx = 0
vy = 0
v = [0,0]
curr_x = 0
curr_y = 0
drop_x = 28.545806
drop_y = 77.272981
dst_x = 100
dst_y = 100
pitch = 0
first_waypt = [0,0]
pehlibar = True
g = 9.8
drop_now = 0
flag = 0

#connection_string = ''
#Set up option parsing to get connection string
"""import argparse  
parser = argparse.ArgumentParser(description='Air Delivery')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()"""

connection_string = '/dev/tty.usbmodem1'
sitl = None


#Start SITL if no connection string specified



# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

vehicle.channels.overrides['5'] = 1000

def getVelocity():
    global vx,vy,v
    x=vehicle.velocity[0]
    y=vehicle.velocity[1]
    vx=math.sqrt(x ** 2)
    vy=math.sqrt(y ** 2)
    v = math.sqrt((vx**2) + (vy**2))

def dist(a,b):
    return math.sqrt((a[1]-b[1])**2+(a[0]-b[0])**2)

def getGPS():
    global curr_x,curr_y,pehlibar,first_waypt,drop_x,drop_y,dst_y,dst_x
    x=vehicle.location.global_relative_frame.lat
    y=vehicle.location.global_relative_frame.lon
    if pehlibar == True:
        first_waypt = [x,y]
        print ("Origin: ",first_waypt)
        dst_x , dst_y = getXY_waypt(drop_x,drop_y,first_waypt[0],first_waypt[1],0,0)
        print ("Drop Point: ",[dst_x,dst_y])
        pehlibar = False
    curr_x,curr_y = getXY_waypt(x,y,first_waypt[0],first_waypt[1],0,0)

def getAltitude():
    global height
    height = vehicle.location.global_relative_frame.alt ;
    if height < 0:
        height = -1*height

def getPitch():
    global pitch
    pitch = vehicle.attitude.pitch
    #print pitch

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

while flag == 0:
    pos_old = [0,0]
    #v = [0,0]
    landing_threshold = 10
    getPitch()
    getAltitude() 
    if pitch < 0:
        vxx = (v[0]*cos(pitch))
        vyy = -1*v[1]*sin(pitch)
        R = (vxx)*(sqrt( vyy*vyy + 2*g*height ) + (vyy))/g
    else:
        vxx = v[0]*cos(pitch)
        vyy = v[1]*sin(pitch)
        R = (vxx)*(sqrt( vyy*vyy + 2*g*height ) - (vyy))/g

    if curr_x != pos_old[0] and curr_y != pos_old[1]:
        v = [curr_x - pos_old[0] , curr_y - pos_old[1]]
        vmag = sqrt((v[0]**2) + (v[1]**2))
        u = [v[0]/vmag , v[1]/vmag]
        predpt = [curr_x + R*u[0], curr_y + R*u[1]]
    else:
        predpt = [curr_x, curr_y]

    check = dist(predpt, [dst_x,dst_y])
    print ("Distance ",check)
    print ("Pitch ",pitch)
    print ("Velocity ",v)
    if check < landing_threshold:
        drop_now = 1

    if drop_now == 1:
        vehicle.channels.overrides['4'] = 2000
        vehicle.channels.overrides['4'] = 1000
        #vehicle.play_tune(args.tune)
        print ("DROPPED")
        flag = 1
    time.sleep(0.1)
    pos_old[0] = curr_x
    pos_old[1] = curr_y
    getGPS()


print("\nClose vehicle object")
vehicle.close()
print("Completed")


