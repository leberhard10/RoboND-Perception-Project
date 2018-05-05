## Project: Perception Pick & Place

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This file will cover the rubric criteria and how each item was addressed. 

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

The work done in Exercise 1 was copied over to the project_template file, and pcd files were saved to verify the variables were valid with the new data. In one terminal the command ```$ roslaunch pr2_robot pick_place_project.launch``` was run in ~/catkin_ws and in another terminal, ```pyon project_template.py ``` was called within the scripts folder.

Input PCL cloud.

![pcl_cloud](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/pcl_cloud.PNG)

Downsampleing Voxel Grid added.

![voxel_downsample](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/voxel_downsampled.PNG)

Pass through filter with the parameters from Exercise 1.

![filter_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/pass_throughfilter_1.PNG)

Pass through with the new axis_min and axis_max values for the pass through filter.

![filter_2](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/pass_throughfilter_2.PNG)

The RANSAC filter with the exercise 1 values removed everything exept the biscuits.

![RANSAC_inliers_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/RANSAC_inliers_1.PNG)

The noise filter had more success capturing more of the objects.

![noise_filter_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/noise_filter_1.PNG)


After double checking the steps to apply the noise filter. It was moved to filter out noise before applying the passthrough filter.

![noise_filter_2](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/noise_filter_2.PNG)

The new pass through point cloud was significantly improved.

![filter_3](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/pass_throughfilter_3.PNG)

And while there is still room for improvement with the RANSAC filter, it is also improved with the noise filter at the beginning of the function.

![RANSAC_2](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/RANSAC_inliers_2.PNG)


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

Same as excercise 1, the default values from exercise 2 and the changes to exercise 1 only captured a portion of the objects.

![pcl_object_1]()



#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



