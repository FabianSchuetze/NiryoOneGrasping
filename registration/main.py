r"""
Tries to register the point cloud with the floor
"""
import open3d as o3d
import numpy as np
PointCloud = o3d.geometry.PointCloud
from registration import find_transform, ransac

def create_box() -> o3d.geometry.PointCloud:
    width = 0.11
    height = 0.165
    depth = 0.045
    box = o3d.geometry.TriangleMesh.create_box(
        width=width, height=height, depth=depth)
    n_points = 10000
    pcd = box.sample_points_poisson_disk(n_points)
    yellow =  np.repeat(np.array([[255., 255., 0]]), n_points, axis=0)
    pcd.colors = o3d.utility.Vector3dVector(yellow / 255)
    return pcd

# def box_points():
    # leftfronttop = [0.0

def load_room() -> o3d.geometry.PointCloud:
    room = o3d.io.read_point_cloud('home_floor.ply')
    return room

def _concat(first: o3d.utility.Vector3dVector, second:
            o3d.utility.Vector3dVector) -> o3d.utility.Vector3dVector:
    merged = np.concatenate([np.array(first), np.array(second)], axis=0)
    return o3d.utility.Vector3dVector(merged)

def merge_point_clouds(first: PointCloud, second: PointCloud) -> PointCloud:
    colors = _concat(first.colors, second.colors)
    points = _concat(first.points, second.points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = points
    pcd.colors = colors
    return  pcd

def pick_points(pcd):
    """
    Pickes the locations of the two points
    """
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    indices = vis.get_picked_points()
    # assert len(indices) % 2 == 0, "pick equal n of points from both objects"
    return indices

def box_points():
    points = np.array(
             [[0, 0.16, 0.045],
              [0., 0.16, 0.0],
              [0.11, 0.16, 0],
              [0.11, 0.16, 0.045],
              [0., 0.0, 0.045],
              [0.0, 0.0, 0.0],
              [0.11, 0.0, 0.0],
              [0.11, 0.0, 0.045]])
    return points

# further = np.eye(4)
# further[:3,:3] = np.array([[[ 0.00201994,  0.00398988,  0.99999   ],
        # [-0.999948  ,  0.0100037 ,  0.00197994],
        # [-0.0099957 , -0.999942  ,  0.00400988]]])
# further[0, 3] = -20
# further[1, 3] = 40
if __name__ == "__main__":
    BOX = create_box()
    ROOM = load_room()
    POINTS1 = np.load('points1.npy')
    POINTS2 = box_points()
    DISTANCE, TRANSFORM = ransac(POINTS1, POINTS2)
