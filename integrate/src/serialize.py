from scipy.spatial.transform import Rotation
import open3d as o3d
import numpy as np
import yaml

def load_poses(path):
    graph = o3d.io.read_pose_graph(path)
    poses = [i.pose for i in graph.nodes]
    return poses

def transfromToDict(transform: np.array):
    rot = Rotation.from_matrix(transform[:3,:3])
    quat = rot.as_quat()
    quaternion = {'w':float(quat[3]), 'x': float(quat[0]), 'y':float(quat[1]),
            'z':float(quat[2])}
    translation = {'x':float(transform[0,3]), 'y':float(transform[1,3]),
                'z':float(transform[2,3])}
    output = {}
    output['quaternion'] = quaternion
    output['translation'] = translation
    return output

def step_to_dict(idx:int, transform: np.array):
    tmp = {}
    tmp['camera_to_world'] = transfromToDict(transform)
    tmp['depth_image_filename'] = "%03d.png" %idx
    tmp['rgb_image_filename'] = "%03d.png" %idx
    tmp['timestamp'] = idx
    return tmp

def poses_to_dict(poses):
    output_dict = {}
    breakpoint()
    for idx, pose in enumerate(poses):
        output_dict[idx] = step_to_dict(idx, pose)
    return output_dict

def save_dict(to_serialize, path):
    with open(path, 'w') as file:
        documents = yaml.dump(to_serialize, file)
    return documents
