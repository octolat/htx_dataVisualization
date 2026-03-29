import time
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as et
import yaml

import rerun as rr
import rerun.blueprint as rrb

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


from helpers import createHeatMap, processUrdf, sceneSetup

# config here
def render(configs):
    #init variables
    bagpath = Path(configs["datapath"])
    print_times = configs["print_times"]
    scene_path = Path(configs["scene_path"])
    urdf_path, gripper_urdf_path = Path(configs["urdf_path"]), Path(configs["gripper_urdf_path"])
    blueprint_path = Path(configs["blueprint_path"])
    point_radii = configs["point_radii"]
    point_colorscheme = configs["point_colorscheme"]
    dof = configs["arm_dof"]

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    #get static data 
    static_data = getMetadataFromBag(bagpath, typestore, dof) #startTime, jointNames, gripperNames, imageSize
    
    #set timeline to the start
    rr.set_time("bag_log_time", timestamp=static_data["startTime"])
    
    # process urdf models
    urdf_data = { 
        "urdf_paths": (urdf_path, gripper_urdf_path),
        "urdf_names": ("arm", "gripper"),
        "driving_joints": (static_data["jointNames"], "finger_joint"), #not a list means it will auto search for mimic joints
        "prefix_names": ("left", "right"),

    }
    tree_joints, paths = processUrdf(urdf_data)

    # set up the scene
    sceneSetup(paths, scene_path)

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
                logArmTransform(tree_joints, msg, "left", dof)
                if print_times: print(f"joints: {time.time()-per}")

            elif connection.topic == '/right_arm/arm_feedback' and True:
                logArmTransform(tree_joints, msg, "right", dof)
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


def logArmTransform(tree_joints, msg, prefix, dof):
    for i in range(dof): #TODO: errr maybe check if names are in order or sum
        pos = msg.position[i]
        if int(msg.name[i][-1]) % 2 == 0: #TODO: super breakable
            # wrapes from 0 - 6.24 to -3.14 to 3.14 for non inf joints
            if pos > 3.14: 
                pos = pos - 6.28
        rr.log(f"{prefix}/transforms", tree_joints[prefix]["arm"][i].compute_transform(pos))
        rr.log(f"{prefix}/jointFeedback/{msg.name[i]}", rr.Scalars(msg.position[i]))

    if len(msg.position) > dof:
        for joint, multiplier, offset in tree_joints[prefix]["gripper"]:
            tf = joint.compute_transform(msg.position[dof] * multiplier + offset)
            rr.log(f"{prefix}/transforms", tf)
        rr.log(f"{prefix}/gripperFeedback/"+msg.name[dof], rr.Scalars(msg.position[dof]))
    else:
        rr.log("alerts", rr.TextLog(f"{prefix} gripper is missing!!"))



def getMetadataFromBag(bagpath, typestore, dof):
    data = {
        "startTime" : None,
        "jointNames" : None,
        "gripperName" : None,
        "imageSize": None,
    }
    data_left = len(data) #TODO: like wth cmon man
    with Reader(bagpath) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            time = np.datetime64(timestamp, "ns")
            # feed in start time
            if data["startTime"] == None:
                data_left -= 1
                data["startTime"] = time
          
            for side in ["left", "right"]:
                if connection.topic == f"/{side}_arm/arm_feedback":
                    # feed in gripper + arm names
                    if data["jointNames"] == None:
                        data_left -= 1
                        data["jointNames"] = msg.name[:dof]

                    if data["gripperName"] == None:
                        if len(msg.name) > dof:
                            data_left -= 1
                            data["gripperName"] = msg.name[dof]
                        

            # feed in image size
            if connection.topic == "/camera/color/image_raw":
                if data["imageSize"] == None:
                    data_left -= 1
                    data["imageSize"] = (msg.height, msg.width)


            #check if all data is extracted
            if data_left <= 0:
                break
        
    return data