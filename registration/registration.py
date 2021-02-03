import numpy as np


def _find_translation(dest: np.ndarray, src: np.ndarray, rotation:
        np.ndarray) -> np.ndarray:
    """
    The translations between the centroids
    """
    centroid_dest = np.mean(dest, axis=0)
    centroid_src = np.mean(src, axis=0)
    translation = centroid_dest - rotation @ centroid_src
    return translation

def is_rotation(rot: np.ndarray) -> bool:
    is_equl = np.isclose(rot.T, np.linalg.inv(rot)).all()
    corret_det = np.isclose(np.linalg.det(rot), 1.0)
    return is_equl and corret_det

def _find_rotation(dest: np.ndarray, src: np.ndarray) -> np.ndarray:
    normalized_dest = dest - np.mean(dest, axis=0)
    normalized_src = src - np.mean(src, axis=0)
    u, _, v = np.linalg.svd(normalized_src.T @ normalized_dest)
    tmp = np.eye(3)
    tmp[2, 2] = np.linalg.det(u @ v.T)
    rot = v @ tmp @ u.T
    assert is_rotation(rot), "Result is not a valid rotation"
    return rot

def find_transform(points1: np.ndarray, points2: np.ndarray) -> np.ndarray:
    assert len(points1) == len(points2), "must have equal number of points"
    rotation = _find_rotation(points1, points2)
    translation = _find_translation(points1, points2, rotation)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform

#maybe give this a try...
def l2_distance(remaining1: np.ndarray, remaining2: np.ndarray, transform:
                np.ndarray) -> float:
    moved = (transform[:3, :3] @ remaining2.T).T + transform[:3, 3]
    distance = np.linalg.norm(remaining1 - moved)
    return distance

def ransac(points1: np.ndarray, points2: np.ndarray) -> np.ndarray:
    distance = 100000
    indices = np.arange(len(points1))
    for _ in range(10):
        to_pick = np.random.choice(indices, 3, replace=False)
        remain = np.setdiff1d(indices, to_pick)
        cur_transform = find_transform(points1[to_pick], points2[to_pick])
        cur_distance = l2_distance(points1[remain], points2[remain], cur_transform)
        print("The current distance is: %.3f" %cur_distance)
        if cur_distance < distance:
            distance = cur_distance
            transform = cur_transform
    return distance, transform
