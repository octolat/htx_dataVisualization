import time
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as et
import yaml

import rerun as rr
import rerun.blueprint as rrb

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# config here
GRIPPER_NAME = "robotiq_2f_85"
def render(configs):
    #init variables
    bagpath = Path(configs["datapath"])
    print_times = configs["print_times"]
    scene_path = Path(configs["scene_path"])
    urdf_path, gripper_urdf_path = Path(configs["urdf_path"]), Path(configs["gripper_urdf_path"])
    blueprint_path = Path(configs["blueprint_path"])
    point_radii = configs["point_radii"]
    point_colorscheme = configs["point_colorscheme"]

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # set up rerun
    rr.init("rerun_gen3_rosbag_parser")
    rr.spawn()

    #make blueprint nice
    rr.log_file_from_path(blueprint_path)

    #get static data 
    static_data = init_getStaticDataFromBag(bagpath, typestore) #startTime, jointNames, gripperNames, imageSize
    #set timeline to the start
    rr.set_time("bag_log_time", timestamp=static_data["startTime"])
    
    # process and log urdf models
    trees, paths = scene_insertUrdf((urdf_path, gripper_urdf_path), ("arm", "gripper"), ("left", "right"))

    tree_joints = {}
    for side in ["left", "right"]:
        tree_joints[side] = {}
        # get arm joints
        tree_joints[side]["arm"] = init_getJoints(trees[side]["arm"], static_data["jointNames"][side])

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
    with Reader(bagpath) as reader:
        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            # print(connection.topic)
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            timestamp = np.datetime64(timestamp, "ns")
            rr.set_time("bag_log_time", timestamp=timestamp)

            #14 seconds
            if connection.topic == '/camera/color/image_raw' and True:
                image = np.array(msg.data).reshape(msg.height, msg.width, 3)
                rr.log("/camera/color/image_raw", rr.Image(bytes=image, datatype="u8", color_model="RGB", height=static_data["imageSize"][0], width = static_data["imageSize"][1]))
                if print_times: print(f"image: {time.time()-per}")

            # #41.4951057434082 seconds, 31 without color, 20 without log
            elif connection.topic == '/camera/depth/color/points'and True:
                result = convertPointCloud(msg, point_colorscheme)
                if point_colorscheme == "heatmap":
                    points, colors = result
                    rr.log("/camera/depth/color/points", rr.Points3D(points, radii=[point_radii]*msg.width, colors=colors))    
                else:
                    points = result
                    rr.log("/camera/depth/color/points", rr.Points3D(points, radii=[point_radii]*msg.width))  
                if print_times: print(f"points: {time.time()-per}")

            #18 seconds
            elif connection.topic == '/left_arm/arm_feedback'and True:
                log_urdfToTransform(tree_joints, msg, "left", static_data["gripperName"]["left"])
                if print_times: print(f"joints: {time.time()-per}")

            elif connection.topic == '/right_arm/arm_feedback' and True:
                log_urdfToTransform(tree_joints, msg, "right", static_data["gripperName"]["right"])
                if print_times: print(f"joints: {time.time()-per}")

            per = time.time()
    print(time.time()-start)
    

            
            
            

def convertPointCloud(msg, colorscheme):
    # TIME = time.time()

    # Determine endianness
    endian = '>' if msg.is_bigendian else '<'

    # Parse fields to get offsets
    field_map = {f.name: f.offset for f in msg.fields}

    # Check if data type is unsupported (currently only float32)
    for field in msg.fields:
        if field.datatype != 7: raise ValueError("pointcloud datatype must be float32")

    # Use numpy stride tricks to read directly from the raw buffer
    # by constructing a dtype that accounts for point_step padding
    full_dtype = np.dtype({
        'names': ['x', 'y', 'z', 'rgb'],
        'formats': [endian + 'f4'] * 4,
        'offsets': [field_map['x'], field_map['y'], field_map['z'], field_map['rgb']],
        'itemsize': msg.point_step  # accounts for any extra fields/padding
    })

    # Parse entire buffer in one shot
    flat_points = np.frombuffer(bytes(msg.data), dtype=full_dtype)

    # Stack and reshape positions to (height, width, 3)
    points = np.stack([flat_points['x'], flat_points['y'], flat_points['z']], axis=-1)
    points = points.reshape(msg.height, msg.width, 3).astype(np.float32)

    # # Reshape rgb to (height, width)
    # colors = flat_points['rgb'].reshape(msg.height, msg.width).astype(np.float32)

    if colorscheme == "heatmap":
        colors = createHeatMap(flat_points['z'], 0.0, 1.5)
        return points, colors
    else:
        return points

def createHeatMap(heights, min, max):
    # normalize to [0, 1] and clamp 
    t = np.clip((heights - min) / (max - min), 0.0, 1.0)

    blue   = (t * 255).astype(np.uint8)
    green = np.zeros(len(t), dtype=np.uint8)   
    red  = ((1.0 - t) * 255).astype(np.uint8)
    colors = np.stack([red, green, blue], axis=1)

    return colors


def log_urdfToTransform(tree_joints, msg, prefix, gripper):
    for i in range(7): #TODO: errr maybe check if names are in order or sum
        pos = msg.position[i]
        if int(msg.name[i][-1]) % 2 == 0: #TODO: super breakable
            # wrapes from 0 - 6.24 to -3.14 to 3.14 for non inf joints
            if pos > 3.14: 
                pos = pos - 6.28
        rr.log(f"{prefix}/transforms", tree_joints[prefix]["arm"][i].compute_transform(pos))
        rr.log(f"{prefix}/jointFeedback/{msg.name[i]}", rr.Scalars(msg.position[i]))

    #TODO: this assumes gripper is present
    if gripper != None:
        for joint, multiplier, offset in tree_joints[prefix]["gripper"]:
            tf = joint.compute_transform(msg.position[7] * multiplier + offset)
            rr.log(f"{prefix}/transforms", tf)
        rr.log(f"{prefix}/gripperFeedback/"+msg.name[7], rr.Scalars(msg.position[7]))


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

def init_getStaticDataFromBag(bagpath, typestore):
    data = {
        "startTime" : None,
        "jointNames" : {"left": None, "right": None},
        "gripperName" : {"left": None, "right": None},
        "imageSize": None,
    }
    data_left = 4
    with Reader(bagpath) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            time = np.datetime64(timestamp, "ns")
            # feed in start time
            if data["startTime"] == None:
                data_left -= 1
                data["startTime"] = time

            # feed in right arm and gripper names
            for side in ["left", "right"]:
                if connection.topic == f"/{side}_arm/arm_feedback":
                    if data["jointNames"][side] == None:
                        data_left -= 1
                        if msg.name[-1] == GRIPPER_NAME:
                            data["jointNames"][side] = msg.name[:-1]
                            data["gripperName"][side] = GRIPPER_NAME
                        else:
                            data["jointNames"][side] = msg.name[:]
                            data["gripperName"][side] = None
                        

            # feed in image size
            if connection.topic == "/camera/color/image_raw":
                if data["imageSize"] == None:
                    data_left -= 1
                    data["imageSize"] = (msg.height, msg.width)


            #check if all data is extracted
            if data_left <= 0:
                break
        
    return data

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
