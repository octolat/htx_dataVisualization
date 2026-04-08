from multiprocessing import process
import time
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as et

import rerun as rr
import rerun.blueprint as rrb

import zarr
import yaml

from helpers import createHeatMap, processUrdf, sceneSetup

def render(configs):
    #init variables
    zarrpath = Path(configs["datapath"])
    print_times = configs["print_times"]
    times_name, joint_names = configs["times_name"], configs["joint_names"]
    episode_select = configs["episode_select"]
    scene_path = Path(configs["scene_path"])
    urdf_path, gripper_urdf_path = Path(configs["urdf_path"]), Path(configs["gripper_urdf_path"])
    point_radii = configs["point_radii"]
    point_colorscheme = configs["point_colorscheme"]


    #set the timeline 
    rr.set_time(times_name, sequence=0)
    
    # process urdf models
    urdf_data = {
        "urdf_paths": (urdf_path, gripper_urdf_path),
        "urdf_names": ("arm", "gripper"),
        "driving_joints": (joint_names, "finger_joint"), #not a list means it will auto search for mimic joints
        "prefix_names": ("left", "right"),

    }
    tree_joints, paths = processUrdf(urdf_data)

    # set up the scene
    sceneSetup(paths, scene_path)

    # for speed tracking
    start = time.time()
    per = start

    # extract from zarr now
    file = zarr.open(zarrpath, mode="r")

    # find range to be rendered
    epi_start, epi_end, epi_length = getEpisodeRange(file, episode_select)
    
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


    # log transforms
    for side, acts in act_dict.items():
        # init the dictionary with the list of keys
        transforms = {
            "translation": [],
            "quaternion": [],
            "child_frame": [],
            "parent_frame": [],
        }
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
                params = unpackTransformObject(joint.compute_transform(joint_act))
                for key, value in params.items():
                    transforms[key].append(value)

            # compute gripper
            for joint, multiplier, offset in tree_joints[side]["gripper"]:
                joint_count +=1
                tf = joint.compute_transform(gripper_act * multiplier + offset)
                params = unpackTransformObject(tf)
                for key, value in params.items():
                    transforms[key].append(value)

        # send to rerun
        rr.send_columns(
            f"{side}/transforms",
            indexes=[rr.TimeColumn(times_name, sequence=times.repeat(joint_count))],
            columns=rr.Transform3D.columns(
                translation=transforms["translation"],
                quaternion=transforms["quaternion"],
                child_frame=transforms["child_frame"],
                parent_frame=transforms["parent_frame"],
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
    colors = createHeatMap(points[:, :, 2], 0.0, 1.5) # only z rows
    
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

        
            
                


    
    
def unpackTransformObject(tf):
    params = {
        "translation": tf.translation.as_arrow_array()[0].as_py(),
        "quaternion": tf.quaternion.as_arrow_array()[0].as_py(),
        "child_frame": tf.child_frame.as_arrow_array()[0].as_py(),
        "parent_frame": tf.parent_frame.as_arrow_array()[0].as_py(),
    }
    return params


def getEpisodeRange(file, episode_select):
    try:
        episodes = file["meta"]["episode_ends"]
    except:
        print("WARNING: episode ends does not exist. assuming episode length of 300.")
        rr.log("alerts", rr.TextLog(f"episode_ends does not exist!!!!! assuming epi length of 300"))
        return (episode_select[0]*300, episode_select[1]*300, (episode_select[1]-episode_select[0])*300)
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




























'''
[sensor_msgs__msg__PointField(name='x', offset=0, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='y', offset=4, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='z', offset=8, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField'), 
sensor_msgs__msg__PointField(name='rgb', offset=16, datatype=7, count=1, INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8, __msgtype__='sensor_msgs/msg/PointField')]
 is_bigendian=False, point_step=20, row_step=9863680, data=array([126,  63,  93, ...,  92,  82,   0], shape=(9863680,), dtype=uint8), is_dense=True, 
'''