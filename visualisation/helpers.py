import numpy as np
import rerun as rr
import xml.etree.ElementTree as et
import yaml

def sceneSetup(urdf_path_dict, scene_path):
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


def createHeatMap(heights, min, max):
    # normalize to [0, 1] and clamp 
    t = np.clip((heights - min) / (max - min), 0.0, 1.0)

    blue   = (t * 255).astype(np.uint8)
    green = np.zeros(blue.shape, dtype=np.uint8)   
    red  = ((1.0 - t) * 255).astype(np.uint8)
    colors = np.stack([red, green, blue], axis=-1)

    return colors

def processUrdf(urdf_data):
    #make urdf like left n right diffed
    paths = {}
    tree_joints = {}
    for side in urdf_data["prefix_names"]:
        paths[side] = {}
        tree_joints[side] = {}
        for path, thing, driving_joints in zip(urdf_data["urdf_paths"], urdf_data["urdf_names"], urdf_data["driving_joints"]):
            # make path
            paths[side][thing] = _addPrefixToUrdf(path, side)

            # get tree objects
            tree = rr.urdf.UrdfTree.from_file_path(paths[side][thing])
            tree_joints[side][thing] = _getTreeJoints(tree, path, driving_joints)

    # get tree objects
    return tree_joints, paths

def _getTreeJoints(tree, path, driving_joints):
    jointlist = []

    if isinstance(driving_joints, str):
        #search for mimics 
        mimic_joints = _discoverMimicJoints(path)[driving_joints]

        # adding main driving joint
        joint = [_getJoints(tree, [driving_joints])[0], 1, 0]
        jointlist.append(joint)
        # adding mirrored joints
        for name, multiplier, offset in mimic_joints:
            joint = [_getJoints(tree, [name])[0], multiplier, offset]
            jointlist.append(joint)

    else:
        # search for joints in the list only
        jointlist = _getJoints(tree, driving_joints)

    return jointlist

def _addPrefixToUrdf(urdf_path, prefix):
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

def _getJoints(urdf_tree, jointNames):
    joints = []
    for i, name in enumerate(jointNames): 
        joints.append(urdf_tree.get_joint_by_name(name))
    return joints

def _discoverMimicJoints(urdf_path):
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
