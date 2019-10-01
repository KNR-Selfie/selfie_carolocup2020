 # Neural Networks - classifier tester

 This branch is mainly focused on gathering information about development of Neural Networks for *Selfie* project. 

 ## Purpose of this branch

 Purpose of this branch is to test compatibility and performance of neural networks combined with computer vision and robotics.

 ## Code

 Current piece of code contains tester for pre-trained classifier working with ROS environment. For now only basic implementation is provided. This means project needs further development. Code is presented as a ROS package where classifier is being loaded, subscriber on topic is being initialised and information about net outcome is being printed as numpy array. In the future more automation processes need to be implemented regarding dynamic image size recognition, providing information about net outcome on specified topic etc. 

 ## Contributing

 All develompent was made under conda environments and will be done in future as no better solution is known at the moment. `environment.yml` file contains all dependencies which can be used to recreate conda env. Typing

```
conda env create -f environment.yml
```

takes care of all magic happening behind the scenes. This configuration provides more or less compatibility with ROS environment. Current version of *Selfie* project is developed under ROS Kinetic, so neural networks are.
The following command makes ROS to publish image on topic:

```
rosrun image_publisher image_publisher path/to/image
```

 ## Testing

 Plan for tests:
 - plug basler camera to private computer and integrate camera with classification node
 - get data from basler on private computer (images to traing net, rosbag to test performance)
 - integrate on car (conda/ROS integration)
 - integrate with entire *Selfie* system (measure performance)

 ## Troubleshooting

 Useful links while managing ROS/conda environments:
 - https://stackoverflow.com/questions/48039563/import-error-ros-python3-opencv
 - https://github.com/keras-team/keras/issues/6462
 - https://github.com/keras-team/keras/issues/13353
