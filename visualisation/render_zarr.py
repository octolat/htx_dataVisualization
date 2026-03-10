import time
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as et
import yaml

import rerun as rr
import rerun.blueprint as rrb

import zarr
import yaml

def render(configs):
    #init variables
    zarrpath = Path(configs["datapath"])
    print_times = configs["print_times"]
    times_name, joint_names = configs["times_name"], configs["joint_names"]
    episode_select = configs["episode_select"]
    scene_path = Path(configs["scene_path"])
    urdf_path, gripper_urdf_path = Path(configs["urdf_path"]), Path(configs["gripper_urdf_path"])
    blueprint_path = Path(configs["blueprint_path"])
    point_radii = configs["point_radii"]
    point_colorscheme = configs["point_colorscheme"]

    # set up rerun
    rr.init("rerun_gen3_rosbag_parser")
    rr.spawn()

    #make blueprint nice
    rr.log_file_from_path(blueprint_path)

    #set the timeline 
    rr.set_time(times_name, sequence=0)
    
    # process and log urdf models
    trees, paths = scene_insertUrdf((urdf_path, gripper_urdf_path), ("arm", "gripper"), ("left", "right"))

    # get joints to use in transforms later
    tree_joints = {}
    for side in ["left", "right"]:
        tree_joints[side] = {}
        # get arm joints
        tree_joints[side]["arm"] = init_getJoints(trees[side]["arm"], joint_names)

        # get gripper joints
        mimic_joints = init_discoverMimicJoints(paths[side]["gripper"])["finger_joint"]
        jointlist = []
        # adding main driving joint
        joint = init_getJoints(trees[side]["gripper"], ["finger_joint"])
        joint.extend([1, 0])
        jointlist.append(joint)
        # adding mirrored joints
        for name, multiplier, offset in mimic_joints:
            joint = init_getJoints(trees[side]["gripper"], [name])
            joint.extend([multiplier, offset])
            jointlist.append(joint)
        tree_joints[side]["gripper"] = jointlist

    # set up the scene
    scene_setup(paths, scene_path)

    start = time.time()
    per = start
    # extract from zarr now
    file = zarr.open(zarrpath, mode="r")

    # find range to be rendered
    epi_start, epi_end, epi_length = init_getEpisodeRange(file, episode_select)
    
    # define timeline column
    times = np.arange(epi_length)


    # log actions
    actions = file["data"]["state"][epi_start:epi_end]
    act_dict = {
        "left": (actions[:, :7], actions[:, -2]), #first 8
        "right": (actions[:, 7:-2], actions[:, -1]), # next 8
    }
    for side, act in act_dict.items():
        arm_act, gripper_act = act
        # log arm
        for i in range(7):
            joint_act = arm_act[:, i]
            rr.send_columns(
                f"/{side}/jointFeedback/{joint_names[i]}",
                indexes=[rr.TimeColumn(times_name, sequence=times)],
                columns=rr.Scalars.columns(scalars=joint_act),
            )

        # log gripper
        rr.send_columns(
            f"/{side}/gripperFeedback/{joint_names[-1]}",
            indexes=[rr.TimeColumn(times_name, sequence=times)],
            columns=rr.Scalars.columns(scalars=gripper_act),
        )
    if print_times: print(f"joint states: {time.time()-per}")
    per = time.time()

    # compute and log transforms
    for side, acts in act_dict.items():
        arm_transforms = gripper_transforms = {}
        for dict in (arm_transforms, gripper_transforms):
            for key in ("translation","quaternion","child_frame","parent_frame"):
                dict[key] = []

        arm_acts, gripper_acts = acts
        for arm_act, gripper_act in zip(arm_acts, gripper_acts):
            
            joint_count = 0
            # compute arm
            for i, joint_act in enumerate(arm_act):
                joint_count += 1
                # wrapes from 0 - 6.24 to -3.14 to 3.14 for non inf joints
                if joint_act % 2 == 0 and joint_act > 3.14: #TODO: super breakable
                    joint_act = joint_act - 6.28
                joint = tree_joints[side]["arm"][i]
                params = init_unpackTransformObject(joint.compute_transform(joint_act))
                for key, value in params.items():
                    arm_transforms[key].append(value)

            # compute gripper
            for joint, multiplier, offset in tree_joints[side]["gripper"]:
                joint_count +=1
                tf = joint.compute_transform(gripper_act * multiplier + offset)
                params = init_unpackTransformObject(tf)
                for key, value in params.items():
                    gripper_transforms[key].append(value)

        # send to rerun
        rr.send_columns(
            f"{side}/transforms",
            indexes=[rr.TimeColumn(times_name, sequence=times.repeat(joint_count))],
            columns=rr.Transform3D.columns(
                translation=arm_transforms["translation"],
                quaternion=arm_transforms["quaternion"],
                child_frame=arm_transforms["child_frame"],
                parent_frame=arm_transforms["parent_frame"],
            ),
        )
    if print_times: print(f"transforms: {time.time()-per}")
    per = time.time()        

    # log pictures
    imgs = file["data"]["img"][epi_start:epi_end]
    height, width, _ = imgs[0].shape
    format = rr.components.ImageFormat(width=width, height=height, color_model="RGB", channel_datatype="U8")
    rr.log("/camera/color/image_raw", rr.Image.from_fields(format=format), static=True)
    rr.send_columns(
        "/camera/color/image_raw",
        indexes=[rr.TimeColumn(times_name, sequence=times)],
        columns=rr.Image.columns(buffer=imgs.reshape(len(times), -1)),
    )
    if print_times: print(f"pictures: {time.time()-per}")
    per = time.time()


    # log point 
    points = file["data"]["point_cloud"][epi_start:epi_end]
    xyz = points[:, :, :3] # array goes [x,y,z, r,g,b]
    colors = createHeatMap(points[:, :, 2], 0.0, 1.5)
    
    rr.send_columns(
        "/camera/depth/color/points",
        indexes=[rr.TimeColumn("tick", sequence=times)],
        columns=[
            *rr.Points3D.columns(positions=xyz.reshape(-1, 3)).partition(lengths=np.full(epi_length, xyz.shape[1])),
            *rr.Points3D.columns(radii=np.full(epi_length, point_radii)),
        ],
    )
    if point_colorscheme == "heatmap":
        rr.send_columns(
            "/camera/depth/color/points",
            indexes=[rr.TimeColumn("tick", sequence=times)],
            columns=[
                *rr.Points3D.columns(colors=colors.reshape(-1, 3)).partition(lengths=np.full(epi_length, xyz.shape[1]))
            ],
        )

    if print_times: print(f"points: {time.time()-per}")
    per = time.time()

    if print_times: print(f"total: {time.time()-start}")

        
            
                


    
    

            


            
   

def createHeatMap(heights, min, max):
    # normalize to [0, 1] and clamp 
    t = np.clip((heights - min) / (max - min), 0.0, 1.0)

    blue   = (t * 255).astype(np.uint8)
    green = np.zeros(blue.shape, dtype=np.uint8)   
    red  = ((1.0 - t) * 255).astype(np.uint8)
    colors = np.stack([red, green, blue], axis=2)

    return colors


def scene_insertUrdf(urdf_paths, names, prefixes):
    #make urdf like left n right diffed
    paths = {}
    trees = {}
    for side in prefixes:
        paths[side] = {}
        trees[side] = {}
        for path, thing in zip(urdf_paths, names):
            # make path
            paths[side][thing] = init_addPrefixToUrdf(path, side)

            # get tree
            trees[side][thing] = rr.urdf.UrdfTree.from_file_path(paths[side][thing])

    return trees, paths


def scene_setup(urdf_path_dict, scene_path):
    #get configs
    with open(scene_path, 'r') as file:
        scene_dict = yaml.safe_load(file)

    # insert urdfs
    for side in urdf_path_dict.values():
        for path in side.values():
            rr.log_file_from_path(path, static = True)

    # set camera frames
    rr.log("/camera/depth/color/points", rr.CoordinateFrame("camera_frame"))
    rr.log("/camera/color/image_raw", rr.CoordinateFrame("camera_frame"))

    # set scene transform for camera 
    rr.log("/scene_transforms", rr.Transform3D(translation=scene_dict["camera"]["translation"], #measured from cad
                                               rotation = rr.RotationAxisAngle(axis=scene_dict["camera"]["rotation_axis"], degrees=scene_dict["camera"]["rotation_degree"]),
                                               child_frame="camera_frame", parent_frame="tf#/"))
    
    # transform for both arms
    rr.log("/scene_transforms", rr.Transform3D(translation=scene_dict["left_arm"]["translation"], #measured from cad
                                               rotation = rr.RotationAxisAngle(axis=scene_dict["left_arm"]["rotation_axis"], degrees=scene_dict["left_arm"]["rotation_degree"]),
                                               child_frame="left_base_link", parent_frame="tf#/"))
    rr.log("/scene_transforms", rr.Transform3D(translation=scene_dict["right_arm"]["translation"], 
                                               rotation = rr.RotationAxisAngle(axis=scene_dict["right_arm"]["rotation_axis"], degrees=scene_dict["right_arm"]["rotation_degree"]),
                                               child_frame="right_base_link", parent_frame="tf#/"))

    # mount gripper to the arm
    rr.log("/scene_transforms", rr.Transform3D(translation=[0.0, 0.0, -0.06149039], #got this number from mujoco_menagerie 
                                               rotation = rr.RotationAxisAngle(axis=(0, 1, 0), degrees=180),
                                               child_frame="right_robotiq_85_base_link", parent_frame="right_bracelet_link"))
    rr.log("/scene_transforms", rr.Transform3D(translation=[0.0, 0.0, -0.06149039], 
                                               rotation = rr.RotationAxisAngle(axis=(0, 1, 0), degrees=180),
                                               child_frame="left_robotiq_85_base_link", parent_frame="left_bracelet_link"))

def init_unpackTransformObject(tf):
    params = {
        "translation": tf.translation.as_arrow_array()[0].as_py(),
        "quaternion": tf.quaternion.as_arrow_array()[0].as_py(),
        "child_frame": tf.child_frame.as_arrow_array()[0].as_py(),
        "parent_frame": tf.parent_frame.as_arrow_array()[0].as_py(),
    }
    return params


def init_addPrefixToUrdf(urdf_path, prefix):
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

def init_getEpisodeRange(file, episode_select):
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
    return (epi_start, epi_end, selected_length)

def init_getJoints(urdf_tree, jointNames):
    joints = []
    for i, name in enumerate(jointNames): 
        joints.append(urdf_tree.get_joint_by_name(name))
    return joints

def init_discoverMimicJoints(urdf_path):
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



























'''
[sensor_msgs__msg__PointField(name='x', offset=0, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='y', offset=4, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='z', offset=8, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='rgb', offset=16, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField')]
 is_bigendian=False, point_step=20, row_step=9863680, data=array([126,  63,  93, ...,  92,  82,   0], shape=(9863680,), dtype=uint8), is_dense=True, 
'''