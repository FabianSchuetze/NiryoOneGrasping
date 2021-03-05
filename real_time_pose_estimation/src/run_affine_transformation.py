import cv2 as cv
import numpy as np
from src.affine_transformation import Camera, GenerateModel
from src.main import load_model


def get_train_model():
    model = load_model('Data/teebox_features.yml')
    model['img'] = cv.imread('Data/teebox_Color.png')
    return model


def get_test_model():
    img = cv.imread(
        '/home/fabian/Documents/work/realsense/data/2021-03-04-17-39/color_imgs/100.png')
    depth = cv.imread(
        '/home/fabian/Documents/work/realsense/data/2021-03-04-17-39/depth_imgs/100.png',
        cv.IMREAD_ANYDEPTH)
    camera = Camera(fx=615.4, fy=614.18, cx=326.27, cy=237.21)
    model = GenerateModel(depth, img, camera)
    orb = cv.ORB_create(nfeatures=1000)
    model.estimate_features(orb)
    model._model['img'] = img
    return model._model


def corresponding_3d_points(train, test, matches):
    points_3d_train, points_3d_test = [], []
    for match in matches:
        point_3d_train = train['points3d'][match.trainIdx]
        point_3d_test = test['points3d'][match.queryIdx]
        points_3d_test.append(point_3d_test)
        points_3d_train.append(point_3d_train)
    return np.array(points_3d_train), np.array(points_3d_test)


def match(train, test):
    bf = cv.BFMatcher(cv.NORM_HAMMING)
    matches = bf.knnMatch(test['descriptors'], train['descriptors'], k=2)
    good = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)
    matches = good
    return matches


if __name__ == "__main__":
    TRAIN = get_train_model()
    TRAIN['points3d'] /= 100
    TEST = get_test_model()
    MATCHES = match(TRAIN, TEST)
    PT_3D_TRAIN, PT_3D_TEST = corresponding_3d_points(TRAIN, TEST, MATCHES)
    trans = cv.estimateAffine3D(PT_3D_TRAIN, PT_3D_TEST, ransacThreshold=0.1)
