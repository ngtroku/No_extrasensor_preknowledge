import numpy as np
import small_gicp

def registration(source, target, registration_type="GICP", downsampling_resolution=0.25): # downsampling_resolution : default = 0.25
    result = small_gicp.align(target, source, registration_type=registration_type, downsampling_resolution=downsampling_resolution)
    return result