import cv2 as cv
import numpy as np
import open3d as o3d
import yaml
from src.affine_transformation import Camera, GenerateModel
from src.main import load_model


def get_train_model(arguments: dict):
    model = load_model(arguments['model_features'])
    model['img'] = cv.imread(arguments['model_img'])
    return model


def point_cloud(color_img, depth_img):
    img = o3d.io.read_image(color_img)
    depth = o3d.io.read_image(depth_img)
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


def get_test_model(arguments: dict):
    # col_loc = '/home/fabian/Documents/work/realsense/data/2021-03-06-17-01/color_imgs/250.png'
    # dep_loc = '/home/fabian/Documents/work/realsense/data/2021-03-06-17-01/depth_imgs/250.png'
    img = cv.imread(arguments['test_color_img'])
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    depth = cv.imread(arguments['test_depth_img'])
    camera = Camera(fx=615.4, fy=614.18, cx=326.27, cy=237.21)
    model = GenerateModel(depth, img, camera)
    orb = cv.SIFT_create()
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
    bf = cv.BFMatcher(crossCheck=False)
    matches = bf.knnMatch(test['descriptors'], train['descriptors'], k=2)
    good = []
    for m, n in matches:
        if m.distance < 0.70 * n.distance:
            good.append(m)
    matches = good
    return matches


def _estimate_rotation(diff_source, diff_target):
    h_mat = diff_source.T @ diff_target
    u, s, v = np.linalg.svd(h_mat)
    rotation = v @ u.T
    if np.linalg.det(rotation) < 0:
        v[:, -2] *= -1
        rotation = v @ u.T
    return rotation


def estimate_transformation(source: np.array, target: np.array):
    breakpoint()
    centroid_source = source.mean(axis=0, keepdims=True)
    centroid_target = target.mean(axis=0, keepdims=True)
    rotation = _estimate_rotation(source - centroid_source,
                                  target - centroid_target)
    translation = centroid_target.T - rotation @ centroid_source.T
    transformation = np.concatenate([rotation, translation], axis=1)
    transformation = np.vstack((transformation, np.zeros((1, 4))))
    transformation[3, 3] = 1
    return transformation


def parse_yml(file: str):
    with open(file) as f:
        data = yaml.safe_load(f)
    return data


if __name__ == "__main__":
    ARGS = parse_yml('Data/parse_python_commands.yml')
    TRAIN = get_train_model(ARGS)
    TRAIN['points3d'] /= 100
    TEST = get_test_model(ARGS)
    MATCHES = match(TRAIN, TEST)
    PT_3D_TRAIN, PT_3D_TEST = corresponding_3d_points(TRAIN, TEST, MATCHES)
    trans = cv.estimateAffine3D(PT_3D_TRAIN, PT_3D_TEST, ransacThreshold=0.009)
    T = trans[1]
    out = (T[:, :3] @ PT_3D_TRAIN.T + T[:, [3]]).T
    transformation = estimate_transformation(out, PT_3D_TEST)
