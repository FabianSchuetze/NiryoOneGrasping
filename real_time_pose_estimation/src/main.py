r"""
A python scirpt to match the points
"""
import cv2
import numpy as np
import matplotlib.pyplot as plt
plt.close('all')


def keypoint(node):
    x = node.at(0).real()
    y = node.at(1).real()
    _size = node.at(2).real()
    _angle = node.at(3).real()
    _response = node.at(4).real()
    _octave = int(node.at(5).real())
    _class_id = int(node.at(6).real())
    kp = cv2.KeyPoint(x, y, _size, _angle, _response, _octave, _class_id)
    return kp


def load_model(path):
    model = dict()
    fs = cv2.FileStorage(path, cv2.FileStorage_READ)
    model['descriptors'] = fs.getNode('descriptors').mat()
    model['points3d'] = fs.getNode('points_3d').mat()
    n = model['points3d'].shape[0]
    model['points3d'] = model['points3d'].reshape(n, 3)
    model['points2d'] = fs.getNode('points_2d').mat()
    n = model['points2d'].shape[0]
    model['points2d'] = model['points2d'].reshape(n, 2)
    node_kps = fs.getNode('keypoints')
    kps = []
    for i in range(node_kps.size()):
        kp = keypoint(node_kps.at(i))
        kps.append(kp)
    model['keypoints'] = kps
    return model


def get_keypoints(img):
    orb = cv2.ORB_create()
    kp, des = orb.detectAndCompute(img, None)
    return kp, des


def corresponding_points(model, keypoints_scene, matches):
    model_3d_points = model['points3d']
    keypoints_model = model['keypoints']
    points_3d, points_2d, keypoints_train, keypoints_test = [], [], [], []
    for match in matches:
        point_3d_model = model_3d_points[match.trainIdx]
        kp_model = keypoints_model[match.trainIdx]
        point_2d_scene = keypoints_scene[match.queryIdx].pt
        kp_test = keypoints_scene[match.queryIdx]
        keypoints_train.append(kp_model)
        keypoints_test.append(kp_test)
        # breakpoint()
        points_3d.append(point_3d_model)
        points_2d.append(point_2d_scene)
    return np.array(points_3d), np.array(
        points_2d), keypoints_train, keypoints_test


def transform(points3d, points2d, intrinsics):
    distcoefs = np.zeros(4)
    rval, rvec, tvec, inliers = \
        cv2.solvePnPGeneric(points3d, points2d, intrinsics,
                            distcoefs,
                            reprojectionError=1.0,
                            flags=cv2.SOLVEPNP_ITERATIVE)
    return rval, rvec, tvec, inliers


def camera():
    mat = np.eye(3)
    fx = 615.4
    fy = 614.18
    cx = 326.27
    cy = 237.21
    mat[0, 0] = fx
    mat[1, 1] = fy
    mat[0, 2] = cx
    mat[1, 2] = cy
    return mat


if __name__ == "__main__":
    MODEL = load_model('Data/teebox_features.yml')
    IMG = cv2.imread("Data/teebox_Color.png")
    IMG_TRAIN = cv2.imread("Data/teebox_Color.png")
    # IMG_TRAIN = cv2.imread('/home/fabian/Pictures/matchsticktable_Color.png')
    KP, DES = get_keypoints(IMG)
    BF = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = BF.knnMatch(DES, MODEL['descriptors'], k=2)
# Apply ratio test
    good = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)
    MATCHES = good
    # MATCHES = BF.match(DES, MODEL['descriptors'])
    POINTS_3D, POINTS_2D, KPS_TRAIN, KPS_TEST = corresponding_points(
        MODEL, KP, MATCHES[: 40])
    INTRINSICS = camera()
    OUT = transform(POINTS_3D, POINTS_2D, INTRINSICS)
    img3 = cv2.drawMatches(
        IMG, KP, IMG_TRAIN, MODEL['keypoints'],
        MATCHES[: 40],
        None)
    plt.imshow(img3)
