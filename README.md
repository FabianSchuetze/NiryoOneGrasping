This repo provides four different grasping technologies with the Niryo One
robot arm. These techniques range from the simple image matching to the more
precise but more involved grasp pose detection.

Installation
------------
The easiest way to work with the repo is to install the docker container.

Image Matching
--------------

Clustering
----------
The advantage of clustering is that (in contrast to the three other techniques
presented) it allows grasping object without having a template of the object.
Whilst this provides more freedom, it also does not allow a full pose
estimation of the object. IS THIS ACTUALLY CORRECT?


Geomertic Matching (Point Cloud Registration)
---------------------------------------------


GPD
---
Interacting with GPD requires the gpd repo as a dependency. GPD consists of two
different repos, the core [GPD module](https://github.com/atenpas/gpd), and the
[GPD-ROS](https://github.com/atenpas/gpd_ros) repo. GPD provides advantageous
graps poses directly on the point cloud of objects. In contrast, the techniques
above grasp objects only from above, this technique can pick an object from
other directions, if these directions are assumed to be more felicitious. To
run GPD launch the following files:

```shell
roslaunch generate_images broadcast_transform.launch
roslaunch integration integration_and_pose_estimation.launch
roslaunch gpd gpd_interaction.launch
```

INCLUDE THE GPD code here as well.
