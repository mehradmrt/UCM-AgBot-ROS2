#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import numpy as np
import pyproj

# Global variables
lat0, lon0 = None, None
path_x, path_y = [], []

# Function to convert latitude and longitude to local x, y coordinates
def latlon_to_xy(lat, lon, lat0, lon0):
    geod = pyproj.Geod(ellps='WGS84')
    fwd_azimuth, back_azimuth, distance = geod.inv(lon0, lat0, lon, lat)
    angle_rad = np.deg2rad(fwd_azimuth)
    x = distance * np.cos(angle_rad)
    y = distance * np.sin(angle_rad)
    return x, y

# Callback for GPS data
def gps_callback(data):
    global lat0, lon0, path_x, path_y
    
    # Save the initial point
    if lat0 is None and lon0 is None:
        lat0, lon0 = data.latitude, data.longitude
    
    # Convert latitude/longitude to x, y
    x, y = latlon_to_xy(data.latitude, data.longitude, lat0, lon0)
    
    # Append to the path list
    path_x.append(x)
    path_y.append(y)

# Main function
def gps_plotter():
    rospy.init_node('gps_plotter', anonymous=True)
    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)
    
    # Wait for GPS data
    rospy.loginfo("Waiting for GPS data...")
    rospy.spin()

    # Plot the path when finished
    plt.figure()
    plt.plot(path_x, path_y, marker='o', color='b')
    plt.title("Robot Path")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    try:
        gps_plotter()
    except rospy.ROSInterruptException:
        pass
