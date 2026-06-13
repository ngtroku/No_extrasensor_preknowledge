#!/usr/bin/env python3

import numpy as np
import rospy

def cartesian2polar(x, y):
    r = (x ** 2 + y ** 2) ** 0.5
    theta = np.degrees(np.arctan2(y, x)) 
    return r, theta

def polar2cartesian(r, theta):
    x = r * np.cos(np.radians(theta))
    y = r * np.sin(np.radians(theta))
    return x, y

def defence_main(raw_points, largest_score_angle, spoofing_range):
    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 

    if temp_min < 0: 
        min = 360 - temp_min
        max = temp_max

    elif temp_max > 360: 
        min = temp_min
        max = temp_max - 360

    else: 
        min = temp_min
        max = temp_max

    r, theta = cartesian2polar(raw_points[:, 0], raw_points[:, 1]) 
    z = raw_points[:, 2]
    mask = ((min <= theta) & (theta <= max)) 

    r_deleted = r[~mask]
    theta_deleted = theta[~mask]
    z_deleted = z[~mask]

    x_spoofed, y_spoofed = polar2cartesian(r_deleted, theta_deleted)
    removed_points = np.vstack((x_spoofed, y_spoofed, z_deleted)).T

    return removed_points