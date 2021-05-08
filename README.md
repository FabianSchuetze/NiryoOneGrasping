This repo provides four different grasping technologies with the Niryo One
robot arm. These techniques range from the simple image matching to the more
precise but more involved grasp pose detection.



Basic Aspects
-------------
Instruction the robot to condut different grasp strategies is very similar.
Each approach requires setting up the camera, some form of graps generation,
and a reception of the grasp frame to imply the robot to move.


Installation
------------
The easiest way to work with the repo is to install the docker container.

Image Matching
--------------
Image matching produces fast and relatively robust results. In constrast to the
three other techniques discussed, image matching does not need to integrate
different RGBD frames into a global scene, but can instead work with one RGB
image. However, the feature detection algorithms are not as robust as the
geometric registration algorithms below, so the quality of the grasp pose is a
bit lower. The instructions to invok grasping based on image matching are:   

```shell
roslaunch generate_images broadcast_transform.launch
roslaunch pose_detection visual_pose.launch
roslaunch new_pick_place picking.launch
```
![GPD](assets/clustering.gif)
As can be seen in the acompanying video, the position estimates are robust, but
the yaw angles flucuate slightly.


Centroid Estimation
----------
The advantage of estimating the centroid of segmented object lies in its
flexibility: Whilst the other techniques presented require a template of the
object, the centroid of a clustered object can be found for objects without
template. The disadvantage of the approach however is that the centroid
represents only a point but no orientation. Thus objects without a pronounced
yaw can be grasped well, but other objects are often not grapsed well.

```shell
roslaunch generate_images broadcast_transform.launch
roslaunch integration integration_and_clustering.launch
roslaunch new_pick_place picking.launch
```


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

```shell
roslaunch generate_images broadcast_transform.launch
roslaunch integration integration_and_pose_estimation.launch
roslaunch new_pick_place picking.launch
```

GPD
---
Interacting with GPD requires the gpd repo as a dependency. GPD consists of two
different repos, the core [GPD module](https://github.com/atenpas/gpd), and the
[GPD-ROS](https://github.com/atenpas/gpd_ros) repo. GPD provides advantageous
graps poses directly on the point cloud of objects. In contrast, the techniques
above grasp objects only from above, this technique can pick an object from
any direction, if these directions are assumed to be more felicitious. To
run GPD launch the following files:

```shell
roslaunch generate_images broadcast_transform.launch
roslaunch integration integration_and_pose_estimation.launch
roslaunch new_pick_place picking.launch
```
A vide of the result can be seen below:  
![GPD](assets/gpd.gif)
At arm moves at first around a pre-specificed trajectory to collect rgbd
images. The images are then registered and integrated into an entire scene.
Then, a geometric registration is identifing the object in the scene. The
identification allows passing the object mesh to GDP and also to locate the
correct reference frame for the object. GDP then suggests different grasps.
Finally, the graps is conducated by the robot. Rviz shows the different
waypoints for the grasping trajectory. Whilst GPD can generate high quality
grasps, approaching the object from another positions than from above requires
a larger workspace for the robot. This requires extra care for selecting a safe
trajectory so that the arm does not collide with the ground, itself of the
object. 













