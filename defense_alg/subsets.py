import small_gicp
import numpy as np

def registration(source, target, registration_type = "PLANE_ICP", downsampling_resolution=0.25): # downsampling_resolution : default = 0.25
    result = small_gicp.align(target, source, registration_type=registration_type,downsampling_resolution=downsampling_resolution)
    return result

# separate points 
def separate_points(pointcloud, n_subset=6):

    horizontal_angles = np.degrees(np.arctan2(pointcloud[:, 1], pointcloud[:, 0]))
    subsets = []
    step = 360.0 / n_subset

    for i in range(n_subset):
        lower_bound = -180.0 + i * step
        upper_bound = -180.0 + (i + 1) * step

        if i == n_subset - 1:
            mask = (horizontal_angles >= lower_bound) & (horizontal_angles <= upper_bound)
        else:
            mask = (horizontal_angles >= lower_bound) & (horizontal_angles < upper_bound)

        subsets.append(pointcloud[mask])

    return subsets

def subset_scan_matching(source_subsets, target_subsets):
  
    deltas = []
    if len(source_subsets) != len(target_subsets):
        print("Error: Number of subsets between source and target is not the same.")
        return deltas

    for i in range(len(source_subsets)):

        if len(source_subsets[i]) < 3 or len(target_subsets[i]) < 3:
            deltas.append((0.0, 0.0))
            continue
            
        res = registration(source=source_subsets[i], target=target_subsets[i])
        deltas.append(res.T_target_source)

    return deltas