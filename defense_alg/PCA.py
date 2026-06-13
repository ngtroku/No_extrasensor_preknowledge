
import numpy as np
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

def extract_features(matrix):
    if isinstance(matrix, tuple):
        return np.eye(4)[:3, :4].flatten()
    return matrix[:3, :4].flatten()

def pca_main(transform_matrices):
    X = np.array([extract_features(m) for m in transform_matrices])
    
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)

    pca = PCA(n_components=2)
    X_pca = pca.fit_transform(X_scaled)
    evr = pca.explained_variance_ratio_

    return X_pca, evr