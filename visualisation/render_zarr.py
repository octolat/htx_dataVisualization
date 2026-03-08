from sys import version
import time
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as et

import rerun as rr
import rerun.blueprint as rrb

import zarr

# config here
print_times = False
zarrfolder_name = "training_data"
episode_select = (0,1) #first episode (inclusive), last episode(exlusive)

# get file paths
zarrpath = Path(__file__).parent / "data" / "zarr_files" / zarrfolder_name

urdf_path = Path(__file__).parent / "resources" / "gen3_urdf" / "urdf" / "gen3.urdf"
gripper_urdf_path = Path(__file__).parent / "resources" / "robotiq-2f-85_urdf" / "urdf" / "robotiq-2f-85.urdf"
blueprint_path = Path(__file__).parent / "resources" / "rerun_gen3_rosbag_parser.rbl"

# set up rerun
rr.init("rerun_gen3_rosbag_parser")
rr.spawn()
# rr.save("output.rrd")

def main():
    # init variables
    times_name = "tick"
    image_size = None
    tree_joints = {
        "left": None,
        "right": None,
    }

    #make blueprint nice
    rr.log_file_from_path(blueprint_path)

    #set the timeline 
    rr.set_time(times_name, sequence=0)
    
    # process and log urdf models
    trees, paths = procAndInsertUrdf(
        (urdf_path, gripper_urdf_path), 
        ("arm", "gripper"), 
        ("left", "right")
        )

    # find mimic joints
    gripper_mimic = {
        "left": discoverMimicJoints(paths["gripper"]["left"]),
        "right": discoverMimicJoints(paths["gripper"]["right"])
    }  
    
    # set camera frames
    rr.log("/camera/depth/color/points", rr.CoordinateFrame("camera_frame"))
    rr.log("/camera/color/image_raw", rr.CoordinateFrame("camera_frame"))

    # set scene transform
    rr.log("/scene_transforms", rr.Transform3D(translation=[0.0, 0.11453, 0.66634], #measured from cad
                                               rotation = rr.RotationAxisAngle(axis=(1, 0, 0), degrees=207.5),
                                               child_frame="camera_frame", parent_frame="tf#/"))
    
    rr.log("/scene_transforms", rr.Transform3D(translation=[-0.016, 0.0, 0.595], #measured from cad
                                               rotation = rr.RotationAxisAngle(axis=(0, 1, 0), degrees=-90),
                                               child_frame="left_base_link", parent_frame="tf#/"))
    rr.log("/scene_transforms", rr.Transform3D(translation=[0.016, 0.0, 0.595], 
                                               rotation = rr.RotationAxisAngle(axis=(1, 0, 1), degrees=180),
                                               child_frame="right_base_link", parent_frame="tf#/"))

    rr.log("/scene_transforms", rr.Transform3D(translation=[0.0, 0.0, -0.06149039], #got this number from mujoco_menagerie 
                                               rotation = rr.RotationAxisAngle(axis=(0, 1, 0), degrees=180),
                                               child_frame="right_robotiq_85_base_link", parent_frame="right_bracelet_link"))
    rr.log("/scene_transforms", rr.Transform3D(translation=[0.0, 0.0, -0.06149039], 
                                               rotation = rr.RotationAxisAngle(axis=(0, 1, 0), degrees=180),
                                               child_frame="left_robotiq_85_base_link", parent_frame="left_bracelet_link"))

    start = time.time()
    # extracting the data
    file = zarr.open(zarrpath, mode="r")

    # find range to be rendered
    episodes = file["meta"]["episode_ends"]
    if episodes.shape[0] < episode_select[1] or episode_select[0] < 0:
        raise ValueError(f"episode_select is out of range, there are only {episodes.shape[0]} episodes")
    
    if episode_select[0] == 0: 
        epi_start = 0
    else: 
        epi_start = episodes[episode_select[0]-1]+1

    total_length = file["data"]["action"].shape[0]
    if episode_select[1] == episodes.shape[0]: 
        epi_end = total_length #note: epi_end is exclusive, points at the idx AFTER the range
    else: 
        epi_end = episodes[episode_select[1]-1]
    selected_length = epi_end - epi_start

    # define time series n transform list
    times = np.arange(selected_length)
    urdf_transform_dict = {
        "left_arm": [],
        "left_gripper": [],
        "right_arm": [],
        "right_gripper": [],
    }

    # log actions
    JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "robotiq_2f_85"]
    actions = file["data"]["state"][epi_start:epi_end]
    arm_act_dict = {
        "left/jointFeedback": actions[:, :7], #first 7
        "right/jointFeedback": actions[:, 8:15], # 9th to 15th
    }
    for entity_path, act in arm_act_dict.items():
        tf = []
        for i in range(7):
            joint_act = act[:, i]
            rr.send_columns(
                f"{entity_path}/{JOINT_NAMES[i]}",
                indexes=[rr.TimeColumn(times_name, sequence=times)],
                columns=rr.Scalars.columns(scalars=joint_act),
            )
            #TODO: compute transform for urdf

    gripper_act_dict = {
        "left/gripperFeedback": actions[:, 7:8], #8th
        "right/gripperFeedback": actions[:, 15:16], #last (16th)
    }
    for entity_path, act in gripper_act_dict.items():
        rr.send_columns(
                f"{entity_path}/{JOINT_NAMES[7]}",
                indexes=[rr.TimeColumn(times_name, sequence=times)],
                columns=rr.Scalars.columns(scalars=act),
            )
        #TODO: compute transform for urdf

    # log pictures
    imgs = file["data"]["img"][epi_start:epi_end]
    height, width, _ = imgs[0].shape
    format = rr.components.ImageFormat(width=width, height=height, color_model="RGB", channel_datatype="U8")
    rr.log("/camera/color/image_raw", rr.Image.from_fields(format=format), static=True)
    rr.send_columns(
        "images",
        indexes=[rr.TimeColumn(times_name, sequence=times)],
        columns=rr.Image.columns(buffer=imgs.reshape(len(times), -1)),
    )

    # log point 
    points = file["data"]["point_cloud"][epi_start:epi_end]
    batch_size, _ = points[0].shape
    xyz = points[:, :, :3] # array goes [x,y,z, r,g,b]
    colors = createHeatMap(points[:, :, 2], 0.0, 1.5)
    
    rr.send_columns(
        "/camera/depth/color/points",
        indexes=[rr.TimeColumn("tick", sequence=times)],
        columns=[
            *rr.Points3D.columns(positions=xyz, radii=np.full(selected_length, 0.0008)),
        ],
    )
    #TODO: add heatmap

    




    # print(time.time()-start)
    


def createHeatMap(heights, min, max):
    # normalize to [0, 1] and clamp 
    t = np.clip((heights - min) / (max - min), 0.0, 1.0)

    blue   = (t * 255).astype(np.uint8)
    green = np.zeros(blue.shape, dtype=np.uint8)   
    red  = ((1.0 - t) * 255).astype(np.uint8)
    colors = np.stack([red, green, blue], axis=-1)

    return colors
            

def addPrefixToUrdf(urdf_path, prefix):
    tree = et.parse(urdf_path)
    root = tree.getroot()

    #rename root attribute (which becomes the parent entitiy path for everything else)
    root.attrib["name"] = f"{prefix}_{root.attrib["name"]}"

    #rename every fucking link (because the link name becomes the frame name for the link)
    for child in root:
        if child.tag == "link":
            child.attrib["name"] = f"{prefix}_{child.attrib["name"]}"
        elif child.tag == "joint":
            child.find('parent').attrib["link"] = f"{prefix}_{child.find('parent').attrib["link"]}"
            child.find('child').attrib["link"] = f"{prefix}_{child.find('child').attrib["link"]}"
    
    new_path = urdf_path.parent / f"{prefix}_{urdf_path.name}"
    tree.write(new_path, encoding='UTF-8', xml_declaration=True)
    return new_path

def logUrdfToTransform(urdf_tree, urdf_joints, gripper_tree, mimic_dict, msg, prefix):
    if urdf_joints == None:
        urdf_joints = [0]*7
        for i in range(7): #TODO errr maybe change this, its liddat cause gripper +1 at the end
            urdf_joints[i] = urdf_tree.get_joint_by_name(msg.name[i])   
    for i in range(7):
        pos = msg.position[i]
        if int(msg.name[i][-1]) % 2 == 0: #TODO: super breakable
            # wrapes from 0 - 6.24 to -3.14 to 3.14 for non inf joints
            if pos > 3.14: 
                pos = pos - 6.28
        rr.log(f"{prefix}/transforms",urdf_joints[i].compute_transform(pos))
        rr.log(f"{prefix}/jointFeedback/"+msg.name[i], rr.Scalars(msg.position[i]))

    #TODO: this assumes gripper is present
    tf = gripper_tree.get_joint_by_name("finger_joint").compute_transform(msg.position[7])
    rr.log(f"{prefix}/transforms", tf)
    rr.log(f"{prefix}/gripperFeedback/"+msg.name[7], rr.Scalars(msg.position[7]))
    # go through mimic joints
    for name, multiplier, _ in mimic_dict["finger_joint"]:
        tf = gripper_tree.get_joint_by_name(name).compute_transform(msg.position[7] * multiplier)
        rr.log(f"{prefix}/transforms", tf)


def discoverMimicJoints(urdf_path):
    mimic_dict = {}
    tree = et.parse(urdf_path)
    root = tree.getroot()

    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic != None:
            parent = mimic.attrib["joint"]
            if not parent in mimic_dict:
                mimic_dict[parent] = []
            #dict value is a list of (child mimic joint name, multipler, offset)
            mimic_dict[parent].append( (joint.attrib["name"], int(mimic.attrib["multiplier"]), int(mimic.attrib["offset"])))

    return mimic_dict

def procAndInsertUrdf(urdf_paths, names, prefixes):
    #make urdf like left n right diffed
    thing_paths = {}
    for thing_path, name in zip(urdf_paths, names):
        versions = {}
        for pre in prefixes:
            versions[pre] = addPrefixToUrdf(thing_path, pre)
        thing_paths[name] = versions.copy()

    # import it to rerun
    thing_trees = {}
    for thing_name, paths in thing_paths.items():
        versions = {}
        for version_name, path in paths.items():
            rr.log_file_from_path(path, static = True)
            versions[version_name] = rr.urdf.UrdfTree.from_file_path(path)
        thing_trees[thing_name] = versions.copy()

    return thing_trees , thing_paths
    



main()






















'''
[sensor_msgs__msg__PointField(name='x', offset=0, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='y', offset=4, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='z', offset=8, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='rgb', offset=16, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField')]
 is_bigendian=False, point_step=20, row_step=9863680, data=array([126,  63,  93, ...,  92,  82,   0], shape=(9863680,), dtype=uint8), is_dense=True, 
'''