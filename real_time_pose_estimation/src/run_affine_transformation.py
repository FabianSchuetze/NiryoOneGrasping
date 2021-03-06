import cv2 as cv
import numpy as np
from src.affine_transformation import Camera, GenerateModel
from src.main import load_model
import open3d as o3d


def get_train_model():
    model = load_model('Data/teebox_features.yml')
    model['img'] = cv.imread('Data/teebox_Color.png')
    return model


def point_cloud(color_img, depth_img):
    img = o3d.io.read_image(color_img)
    depth = o3d.io.read_image(depth_img)
    breakpoint()
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        img, depth, convert_rgb_to_intensity=False)
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
    intrinsic = camera()
    cam = o3d.camera.PinholeCameraIntrinsic()
    cam.intrinsic_matrix = intrinsic
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)
    return pcd

def get_test_model():
    col_loc = '/home/fabian/Documents/work/realsense/data/2021-03-04-17-39/color_imgs/100.png'
    dep_loc = '/home/fabian/Documents/work/realsense/data/2021-03-04-17-39/depth_imgs/100.png'
    img = cv.imread(col_loc)
    depth = cv.imread(dep_loc, cv.IMREAD_ANYDEPTH)
    camera = Camera(fx=615.4, fy=614.18, cx=326.27, cy=237.21)
    model = GenerateModel(depth, img, camera)
    orb = cv.ORB_create(nfeatures=1000)
    model.estimate_features(orb)
    model._model['img'] = img
    model._model['pcd'] = point_cloud(col_loc, dep_loc)
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
    trans = cv.estimateAffine3D(PT_3D_TRAIN, PT_3D_TEST, ransacThreshold=0.05)
