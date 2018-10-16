# Simple implementation of Dense Visual Odometry with RGB-D camera
![demo](https://user-images.githubusercontent.com/15071493/45149269-1c58e000-b204-11e8-93a9-29b227416191.gif)  
This ROS package provides simple implementation of Dense Visual Odometry with RGB-D camera (https://vision.in.tum.de/_media/spezial/bib/kerl13icra.pdf&hl=ja&sa=X&scisig=AAGBfm147Qu9xs7R6FGEiy3zbmEXLYgvbw&nossl=1&oi=scholarr).  
Dataset is available from https://vision.in.tum.de/data/datasets/rgbd-dataset/download# .  
~~This implementation is just for dense tracking. Robust weighting is not implemented yet.~~

# Dependency
Ubuntu 14.04 LTS or 16.04  
ROS indigo/kinetic  
OpenCV  
PointCloudLibrary  

# Example
* Dowload sample rosbag file https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz  
* Run ```rosrun simple_dvo main /camera/rgb/image_color /camera/depth/image /camera/rgb/camera_info```  
* Run```rosbag play rgbd_dataset_freiburg2_desk.bag```  
* Run ```rosrun rviz rviz```  
* In rviz "Fixed Frame" tab, change "world" to "cam_origin"  
* "Add" -> "By topic" and add "PointCloud2"  
* "Add" -> "By display type" and add "TF"  
  
You can see the camera pose and pointcloud in rviz.  

# References
- Robust Odometry Estimation for RGB-D Cameras (C. Kerl, J. Sturm, D. Cremers), In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), 2013  
- Dense Visual Odometry (https://github.com/tum-vision/dvo)  
