# visualiser
a python script to visualise rosbag2 and zarr files with rerun.

## Building the docker container
in the ```htx_visualisation``` folder run ```docker build -t rerun:0.30.0 .```

## Running the visualiser
1. Edit the ```./run_visualiser.sh``` file to change the data volume mount location to where your data is. (hint: 2nd last line)
2. Edit the ```config.yaml``` (mostly look at the datatype, datapath, and episode_select (for zarr only) parameters)
3. In the ```htx_visualisation``` folder run ```./run_visualiser.sh```
4. press enter to quit

## info
### config file parameters
`datatype`: either "zarr" or "rosbag"  
`datapath`: path from ./visusalisation to your data. should be a folder for zarr, and a bagfile for rosbags  
`print_times`: whether to print debugging info about the amount of time logging took  

-- this section is only used when using processing zarr files --  
`episode_select`: an array [first episode, last episode+1] to choose the range of episodes to view
`times_name`: name of the timeline   
`joint_names`: an array consiting of the joint names + gripper name. used to find arm joints in urdf, as well as to define entitiy paths for each arm joint as well as gripper  
-- this section is only used when using processing zarr files --  

`scene_path`: path from ./visusalisation to yaml decribing the transform of the arm n camera  
`urdf_path`: path from ./visusalisation to arm urdf  
`gripper_urdf_path`: path from ./visusalisation to gripper urdf  
`blueprint_path`: path from ./visusalisation to blueprint file (.rbl)  

`point_radii`: size of the points of the pointcloud in m  
`point_colorscheme`: color scheme of the points. mono makes it all green, heatmap makes it blue to red based on z height  

### supported files
#### rosbags 
supports the following topics
- /camera/color/image_raw (sensor_msgs/msg/image)
- /camera/depth/color/points (sensor_msgs/msg/pointcloud2)
- /left_arm/arm_feedback (sensor_msgs/msg/jointstate)
- /right_arm/arm_feedback (sensor_msgs/msg/jointstate)
uses bagger log time as the timeline
uses row by row logging (instead of zarr's columnlar logging) so a little slower

#### zarr files
must follow anzac's convertor's format  
~ denotes number of episodes  
```
/
├── data
│   ├── action (~, 12) float32
│   ├── img (~, {height}, {width}, 3) uint8
│   ├── point_cloud (~, {points}, 6) float32
│   └── state (~, 12) float32
└── meta
  └── episode_ends (~,) int64
```  