#!/usr/bin/env python3

import numpy as np
import json

def cartesian2polar(x, y):
    r = (x ** 2 + y ** 2) ** 0.5
    theta = np.degrees(np.arctan2(y, x)) 
    return r, theta

def polar2cartesian(r, theta):
    x = r * np.cos(np.radians(theta))
    y = r * np.sin(np.radians(theta))
    return x, y

def decide_spoofing_param(odom_x, odom_y, spoofer_x, spoofer_y):
    spoofing_angle = np.degrees(np.arctan2(spoofer_y - odom_y, spoofer_x - odom_x)) 
    return spoofing_angle

def decide_mask(horizontal_angle, largest_score_angle, spoofing_range):

    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 

    temp_min = (temp_min + 180) % 360 - 180
    temp_max = (temp_max + 180) % 360 - 180

    if temp_min > temp_max:
        spoofing_condition = ((horizontal_angle >= temp_min) | (horizontal_angle <= temp_max))
    else:
        spoofing_condition = ((horizontal_angle >= temp_min) & (horizontal_angle <= temp_max))
    
    return spoofing_condition

def noise_simulation(raw_points, largest_score_angle, spoofing_range):
    rng = np.random.default_rng() 
    with open('config.json', 'r') as f:
        config = json.load(f)

    horizontal_resolution = config['generate_pointcloud']['horizontal_resolution']
    vertical_lines = config['generate_pointcloud']['vertical_lines']
    spoofing_rate = config['generate_pointcloud']['spoofing_rate']

    r, theta = cartesian2polar(raw_points[:, 0], raw_points[:, 1]) 
    z = raw_points[:, 2]

    mask = decide_mask(theta, largest_score_angle, spoofing_range)

    r_deleted = r[~mask]
    theta_deleted = theta[~mask]
    z_deleted = z[~mask]

    num_spoofed_points = int((spoofing_range / horizontal_resolution) * vertical_lines * spoofing_rate)

    r_noise = rng.uniform(0.0, 200.0, num_spoofed_points)
    theta_noise = rng.uniform(largest_score_angle - (spoofing_range / 2), largest_score_angle + (spoofing_range / 2), num_spoofed_points)

    theta_noise = (theta_noise + 180) % 360 - 180
    z_noise = r_noise * np.sin(np.degrees(rng.uniform(-15.0, 15.0, num_spoofed_points)))

    x_remaining, y_remaining = polar2cartesian(r_deleted, theta_deleted)
    points_remaining = np.vstack((x_remaining, y_remaining, z_deleted)).T

    x_spoofed, y_spoofed = polar2cartesian(r_noise, theta_noise)
    points_spoofed = np.vstack((x_spoofed, y_spoofed, z_noise)).T
   
    return points_remaining, points_spoofed

def spoof_main(pointcloud, largest_score_angle, spoofing_range): 
    points_remaining, points_spoofed = noise_simulation(pointcloud, largest_score_angle, spoofing_range)
   
    x_remaining, y_remaining, z_remaining = points_remaining[:, 0], points_remaining[:, 1], points_remaining[:, 2]
    x_spoofed, y_spoofed, z_spoofed = points_spoofed[:, 0], points_spoofed[:, 1], points_spoofed[:, 2]

    return x_remaining, y_remaining, z_remaining, x_spoofed, y_spoofed, z_spoofed
    