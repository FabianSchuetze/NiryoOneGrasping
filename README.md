This repo provides four different grasping technologies with the Niryo One
robot arm. These techniques range from the simple image matching to the more
precise but more involved grasp pose detection.

Installation
------------
The easiest way to work with the repo is to install the docker container.

Image Matching
--------------


Centroid Estimation
----------
The advantage of estimating the centroid of segmented object lies in its
flexibility: Whilst the other techniques presented require a template of the
object, the centroid of a clustered object can be found for all kinds of
objects. The disadvantage of the approach however is that the centroid
represents only a point but no orientation. Thus objects without a pronounced
yaw can be grasped well, but other objects are often not grapsed well.



Geomertic Matching (Point Cloud Registration)
---------------------------------------------
This approach allows computing grasp poses based in geometric matching of a 3D
template with the segmented scene through ICP. The grasps position are in the
center of the object and the grasp are approached in the negative x-direction
of the grasp frame and the positive z-direction of the graps frame. Grasp
orientations are indenpendet of the object shape and the gripper always grasps
the obejct "from above". This makes grasping robust and leds to few collisions
with other objects and the gripper when attempting to grasp a particular
object. However, for objects which are not rectangular, the clusure from above
might not be very firm, and object might slip out of the gripper's hand.



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


