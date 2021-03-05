import copy
from dataclasses import dataclass
import cv2 as cv
import numpy as np


@dataclass
class Camera:
    fx: float
    fy: float
    cx: float
    cy: float


def convert_to_pointcloud(depth_img, camera: Camera):
    points = np.zeros((depth_img.size, 3))
    height, width = depth_img.shape
    idx = 0
    for i in range(height):
        for j in range(width):
            depth = depth_img[i, j] / 1000
            x = (j - camera.cx) * depth / camera.fx
            y = (i - camera.cy) * depth / camera.fy
            points[idx] = [x, y, depth]
            idx += 1
    return points


class GenerateModel:

    def __init__(
            self,
            depth_img: np.array,
            color_img: np.ndarray,
            camera: Camera):
        self._depth_img = depth_img
        self._color_img = color_img
        self._camera = camera
        self._model = dict()

    def _write_to_model(self, kps, des) -> None:
        self._model['keypoints'] = kps
        self._model['points2d'] = np.array([i.pt for i in kps])
        self._model['descriptors'] = des

    def _convert_3d_points(self) -> None:
        breakpoint()
        points = np.zeros((len(self._model['descriptors']), 3))
        for idx, point in enumerate(self._model['points2d']):
            width, height = int(np.round(point[0])), int(np.round(point[1]))
            p_z = self._depth_img[height, width] / 1000
            p_x = (height - self._camera.cx) * p_z / self._camera.fx
            p_y = (width - self._camera.cy) * p_z / self._camera.fy
            points[idx] = [p_x, p_y, p_z]
        self._model['points3d'] = points

    def estimate_features(self, detector: cv.ORB) -> None:
        """
        Estimates key keypoints for the image
        """
        kps, des = detector.detectAndCompute(self._color_img, None)
        self._write_to_model(kps, des)
        self._convert_3d_points()

    def model(self):
        return copy.deepcopy(self._model)
