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

The work done in Exercise 1 was copied over to the project_template file, and pcd files were saved to verify the variables were valid with the new data. In one terminal the command ```$ roslaunch pr2_robot pick_place_project.launch``` was run in ~/catkin_ws and in another terminal, ```pyon project_template.py ``` was called within the scripts folder. At fist pcd files were used to verify the exercise one values, but by exercise 2 it was realized these do not provide the right view. Instead, the data was routed to the point cloud topics to get the camera view of the robot. The assesments are starting with commit 1947993d67d159f1c2e7d2a7b9d5434587c673de "Reroute the raw pcl_cloud to /pclobjects".

Input PCL cloud.

![pcl_cloud](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/pcl_cloud.PNG)

Noise filter applied immediatly applied after ros to pcl conversion with the exercise 1 default parameters.

![noise_filter_d](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/noise_filter_defaults.PNG)

Updated the mean points and the threshold for a cleaner image.

![noise_filter_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/noise_filter_1.PNG)

Applied voxel grid downsampling with the exercise 1 default parameters.

![vox_filter_d](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/vox_filter_defaults.PNG)

Updated the leaf value for a cleaner image. Larger values resulted in a fuzzier image, while smaller values provided less noise in the resulting cloud data.

![vox_filter_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/vox_filter_1.PNG)

Applied pass through filter with the exercise 1 default parameters. It looks like the default axis min and max from exercise 1 filter the correct region of interest.

![pas_filter_d](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/pas_filter_defaults.PNG)

Applied exercise 1 default parameters to the RANSAC filter. It looks like some of the table is captured with the default axis min and max from the passthrough filter.

![RANSAC_filter_d](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/RANSAC_filter_defaults.PNG)

Experimenting with the min xis for the pass through would filter out the table, so the original value was kept and the focus moved to the RANSAC max distance. Decreasing the value captured more of the table and increasing the value started to filter out the objects.

![RANSAC_filter_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/RANSAC_filter_1.PNG)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

On the first try when using PCL Vierwer, only half of the objects were displayed in the pcl cluster. Redoing the exercise one componenst using RViz, the values obtained from excercise 2 successfully recognised object clusters.

![pcl_object_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/pcl_cluster_1.PNG)



#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

To start porting over Exercise 3 code, the training.launch file was copied over from the Exercise 3 launch folder into the pr2_robot launch folder and capture_features.py was also copied over into the scripts folder.

The first run resulted in soap being recognized as soap 2 over half the time. The current training values also resulted in an accuracy score of 0.6111

![accuracy_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/classifier_accuracy_1.PNG)

Continuing on to see what the results are, the bins were mistakenly labeled along with every item labeled soap2.

![labels_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_1.PNG)

The number of attempts to gain a valid point cloud appear to help with the accuracy. It was also noticed that running capture_features.py several times without changing the file will also produce different results. It took about 3 runs to get a perfect detection of the three items. 

![accuracy_2](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/classifier_accuracy_2.PNG)

Despite this, the biscuits are still labeld as soap and the bins are also labeled.

![labels_2](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_2.PNG)

After the YAML file output was added in, the object recognition was revisited to fix the buiscuits and the bins being improperly detected.

![labels_3](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_3.PNG)

To start, the list of object features was expanded to the full list. Otherwise, new model.sav files would need to be generated for each output file. The first round of training with the parameters from list 1, resulted in an accuracy score of 0.70. A second run resulted in 0.775.

![accuracy_3](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/classifier_accuracy_3.PNG)

Out of curiosity, world 3 was run with this and resulted in the following labels.

![labels_4](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_4.PNG)

The new training data with world 1, did succed in recognizing biscuits, but the soap was labeled the same.

![labels_5](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_5.PNG)

Afer several iterations attempting to figure out why the training data accuracy was between .7 to 1.0 and the labels were still invalid. There was suspicion back during the exercises that the normal histogram calculations may not be correct (especially without a lesson that tested the code). After several iteration, it appward the max and min ranges needed to be consistent. The values were polled and new min/max values of -1.0 to 1.0 were added. After the 32 bin size resulted in no change, the number was decreased to 10. After the frustration of very little improvement, it was decided to attempt taking the 0.7 accuracy and test the labels.

![accuracy_5](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/classifier_accuracy_5.PNG)

For the first time, all of the objects (excluding the bins) were accuratly labeled for world 1.

![labels_5](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_5.PNG)

THis wasn't enough for world 2 to pass. it mislabeled 3 out of 5 items. More parameters in the histograms were updated to obtain an accuracy score of 0.875. 

![accuracy_5](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/classifier_accuracy_5.PNG)

Continuing on to world 2 and world 3, the results were not a passing score. It also appeared that as the confusion matrix became more accurate, the object detection became worse. The normal histogram calculations were removed to see if this was part of the problem. Turns out that a passong score for world 3 could be obtained with just the hsv histograms. 

![labels_7](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_7.PNG)

100% accuracy for world 2

![labels_8](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_8.PNG)

The focus returned to the histogram normals. THe valid point cloud checks were restored to the last best fit value and the range of the histograms were extended by 0.1 and resulted in a training set accuracy score of 0.7. All items were accuratly detected in world 1.

![labels_10](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_10.PNG)

This resulted in 4 out of 5 samples for world 2.

![labels_9](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_9.PNG)

6 out of 8 samples were accuratly detected for world 3.

![labels_11](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/object_labels_11.PNG)



### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

Even though biscuits are being detected as soap, the decision ws to keep going with the remaining requirements and fix the object recognition after adding in the yaml file export. On the first test run without runtime errors, soap 2 was accurately detected and grabbed by the correct arm. It was also noticed that soap2 was supposed to be selected last.

![yaml_1](https://github.com/leberhard10/RoboND-Perception-Project/blob/master/images/yaml_output_1.PNG)


### Conclusion

The code utilized the work developed from the exercises and the debugging of the system involved testing each step of the object recognition pipeline. As each section worked as expected, the next step was added back in. When the labels failed to accuratly detect the object, the exercise 3 components were evaluated as mentioned above to determine the current code values.

So far objects that are similar in size like the snacks and the books will be mistaken for each other and the snacks go undetected. Since the normal histograms were the last component, the normal histogram bins as well as the hsv bin numbers could be altered to improve the accuracy. THe next stage would be to obtain the 100% accurate labels in the tabletop challenge world.


