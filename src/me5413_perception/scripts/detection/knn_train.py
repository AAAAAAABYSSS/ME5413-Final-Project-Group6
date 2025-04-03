#!/usr/bin/env python3
import os
import cv2
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.externals import joblib  
from skimage.feature import hog

def extract_hog_features(img):
    img = cv2.resize(img, (28, 28))
    return hog(img, pixels_per_cell=(4, 4), cells_per_block=(2, 2), feature_vector=True)

def load_data(data_dir):
    X, y = [], []
    for label in os.listdir(data_dir):
        class_dir = os.path.join(data_dir, label)
        for file in os.listdir(class_dir):
            img = cv2.imread(os.path.join(class_dir, file), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            features = extract_hog_features(img)
            X.append(features)
            y.append(int(label))
    return np.array(X), np.array(y)

X, y = load_data("dataset/train")
print("Training KNN...")
model = KNeighborsClassifier(n_neighbors=3)
model.fit(X, y)

#  Save
joblib.dump(model, "knn_digit_model.pkl")
print("Model saved as knn_digit_model.pkl")
