from typing import Tuple
import cv2 as cv
import numpy as np
import open3d as o3d
import yaml
import tf
import rospy
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr
from pytransform3d.transform_manager import TransformManager
from src.affine_transformation import Camera, GenerateModel
from src.main import load_model

def lookupTransform():
    rospy.init_node('tf_P3AT')
    t = tf.TransformListener()
    rate = rospy.Rate(1.0)
    for i in range(20):
        try:
            pose = t.lookupTransform("world", "camera_link", rospy.Time(0.0))
            break
        except (tf.LookupException, tf.ConnectivityException):
            rate.sleep()
    if i == 19:
        raise "Could Not find transform"
    return pose


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
    img = cv.imread(arguments['test_color_img'])
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    depth = cv.imread(arguments['test_depth_img'], cv.IMREAD_ANYDEPTH)
    camera = Camera(fx=615.4, fy=614.18, cx=326.27, cy=237.21)
    model = GenerateModel(depth, img, camera)
    orb = cv.SIFT_create()
    model.estimate_features(orb)
    model._model['img'] = img
    model._model['pcd'] = point_cloud(arguments['test_color_img'], arguments['test_depth_img'])
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

def average_distance(source, target):
    return np.sum(np.abs(source - target), axis=0) / len(source)


def convert_points(source, estimation: Tuple):
    """
    Obtain the points from source and target that also match the correspondence
    after estiamtion
    """
    corr = estimation[2]
    source = source[corr.squeeze()]
    return source

def remove_duplicate_point(points):
    final_points = []
    for idx in range(len(points)):
        point = points[idx]
        distances = np.sum(np.abs(points[idx + 1:] - point), axis=1)
        if np.all(distances > 1e-6):
            final_points.append(point)
    return np.array(final_points)

def average_point(points):
    return np.mean(points, axis=0)

def final_transform(pose: np.ndarray):
    tm = TransformManager()
    object2cam = pt.transform_from_pq(np.hstack((AVG, pr.q_id)))
    cam2base = lookupTransform()
    cam2base = pt.transform_from_pq(np.hstack((cam2base[0], cam2base[1])))
    tm.add_transform("world", "camera_link", cam2base)
    tm.add_transform("camera_link", "object", object2cam)
    world2object = tm.get_transform("world", "object")
    print(world2object)
    return world2object

if __name__ == "__main__":
    ARGS = parse_yml('Data/parse_python_commands.yml')
    TARGET = get_train_model(ARGS)
    TARGET['points3d'] /= 100
    SOURCE = get_test_model(ARGS)
    MATCHES = match(TARGET, SOURCE)
    PT_3D_TARGET, PT_3D_SOURCE = corresponding_3d_points(TARGET, SOURCE, MATCHES)
    ESTIMATION = cv.estimateAffine3D(PT_3D_TARGET, PT_3D_SOURCE,
                                     ransacThreshold=0.009)
    ESTIMATED_LOCATION = convert_points(PT_3D_SOURCE, ESTIMATION)
    SINGLE_POINTS = remove_duplicate_point(ESTIMATED_LOCATION)
    AVG = average_point(SINGLE_POINTS)
    final_transform(AVG)
