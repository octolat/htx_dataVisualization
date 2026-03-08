entitiy paths  
** URDFS **
left arm+gripper: left/transforms
right arm+gripper: right/transforms

** Joint Feedback (state) **
left arm: left/jointFeedback/joint_{jointnumber}
right arm: right/jointFeedback/joint_{jointnumber}
left gripper: left/gripperFeedback
right gripper: right/gripperFeedback

** Joint Targets (actions) **
not implemented

** Camera **
rgb camera: /camera/color/image_raw
depth camera: /camera/depth/color/points

** Scene set up **
timeline: bag_log_time
transformations: /scene_transforms




dependencies
rerun-sdk
rosbags
zarr