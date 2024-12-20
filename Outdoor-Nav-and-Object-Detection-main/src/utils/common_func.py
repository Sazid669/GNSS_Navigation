#!/usr/bin/env python3

import math
from math import radians as deg2rad
import utm 
from sensor_msgs.msg import NavSatFix, Imu
import tf


class CommonFunc:
    def __init__(self):
        pass

    def gps_to_cartesian(self,latitude, longitude):
        X, Y, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
        return X, Y

    def wrap_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def angle(self, x1, y1, x2, y2):
        return math.degrees(math.atan2(y2 - y1, x2 - x1))

    def GPS2UTM(self, lat: float, lon: float):
        currentUTM = utm.from_latlon(lat, lon)
        
        return currentUTM[0], currentUTM[1] 
       
    def UTM2ODOM(self, lat: float, lon: float, start_lat: float, start_lon: float, start_ori: float):
        currentUTM = utm.from_latlon(lat, lon)
        startUTM = utm.from_latlon(start_lat, start_lon)
        
        eastDelta = currentUTM[0] - startUTM[0]
        northDelta = currentUTM[1] - startUTM[1]
        x = math.cos(math.radians(start_ori)) * eastDelta + math.sin(math.radians(start_ori)) * northDelta
        y = -math.sin(math.radians(start_ori)) * eastDelta + math.cos(math.radians(start_ori)) * northDelta
        return x, y