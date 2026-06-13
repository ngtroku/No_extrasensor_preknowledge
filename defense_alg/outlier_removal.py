
import numpy as np

from scipy.stats import gaussian_kde

def kde_2d(pca_transform_distribution):

    kde = gaussian_kde(pca_transform_distribution.T)
    return kde

def get_KDEpeak(kde, pca):
    x_min, x_max = pca[:, 0].min() - 1.0, pca[:, 0].max() + 1.0
    y_min, y_max = pca[:, 1].min() - 1.0, pca[:, 1].max() + 1.0

    xx, yy = np.meshgrid(np.linspace(x_min, x_max, 100),
                         np.linspace(y_min, y_max, 100))
    
    grid_coords = np.vstack([xx.ravel(), yy.ravel()])
    Z = kde(grid_coords).reshape(xx.shape)

    row, col = np.unravel_index(np.argmax(Z), Z.shape) 
    kde_peak_x = xx[row, col]
    kde_peak_y = yy[row, col]

    return kde_peak_x, kde_peak_y

def euclidian_distance_filter(current_subsets, pca_transform_distribution, kde_peak_x, kde_peak_y, distance_threshold=2.0):

    diff_x = pca_transform_distribution[:, 0] - kde_peak_x
    diff_y = pca_transform_distribution[:, 1] - kde_peak_y
    euclidian_distance = (diff_x ** 2 + diff_y ** 2) ** 0.5

    outlier_mask = euclidian_distance > distance_threshold

    valid_points = []
    for i, sub_points in enumerate(current_subsets):

        if not outlier_mask[i] and len(sub_points) > 0:
            valid_points.append(sub_points)
            
    if len(valid_points) == 0:
        print("[Warning] All subsets are detected as outliers. Returning an empty pointcloud.")
        return np.empty((0, 3))

    cleaned_pointcloud = np.vstack(valid_points)

    return cleaned_pointcloud

