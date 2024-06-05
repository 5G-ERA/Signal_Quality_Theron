#!/bin/sh

# Check if the library is installed, if not, install it

#SOURCE THE ROS INSTALL
#source /opt/ros/humble/setup.bash
#export ROS_DOMAIN_ID=34
#export ROBOT_TOPIC_POSITION="/robot/amcl_pose"

export USE_MIDDLEWARE=true
#export USE_MIDDLEWARE=false
#export NETAPP_ADDRESS=http://middleware.5gera.robotnik.cloud:5896
export MIDDLEWARE_ADDRESS=middleware.5gera.robotnik.cloud
#export MIDDLEWARE_ADDRESS=middleware-lb-52eed9980d2c0a8a.elb.eu-west-2.amazonaws.com
#export MIDDLEWARE_ADDRESS=crop.5gera.net
export MIDDLEWARE_USER=ad20f254-dc3b-406d-9f15-b73ccd47e867
export MIDDLEWARE_PASSWORD=middleware
#signal
export MIDDLEWARE_TASK_ID=5366890a-cc9b-4316-8b5a-e09049eaa614
#reference
#export MIDDLEWARE_TASK_ID=6d7b20ad-8459-4f00-91f4-816dded6395c
export MIDDLEWARE_ROBOT_ID=300c719a-1c06-4500-a13a-c2e20592b273
# TOPICS TO SEND FROM ROBOT TO NETWORK APPLICATION
export TOPICS_TO_SERVER='[{"name":"/robot/pose","type":"geometry_msgs/msg/PoseWithCovarianceStamped","qos":{"durability":"TRANSIENT_LOCAL","reliability":"RELIABLE","history":"KEEP_LAST","depth":1}},{"name":"/pcl_colour","type":"std_msgs/msg/String"}]'
# TOPICS TO SEND FROM NETWORK APPLICATION BACK TO ROBOT
export TOPICS_FROM_SERVER='[{"name": "/semantic_pcl","type": "sensor_msgs/msg/PointCloud2"}]'
# RUN RELAY CLIENT
python3 -m era_5g_relay_network_application.client

