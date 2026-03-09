import xml.etree.ElementTree as et
from pathlib import Path

import rerun as rr

urdf_path = Path(__file__).parent / "gen3_urdf" / "urdf" / "gen3.urdf"

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

# set up rerun
rr.init("rerun_gen3_rosbag_parser")
rr.spawn()

rr.log_file_from_path(addPrefixToUrdf(urdf_path, "left"), static = True)
rr.log_file_from_path(addPrefixToUrdf(urdf_path, "right"), static = True)

rr.log("/scene_transforms", rr.Transform3D(translation=[0.016, 0.0, 0.595], 
                                               rotation = rr.RotationAxisAngle(axis=(1, 0, 1), degrees=180),
                                               child_frame="left_base_link", parent_frame="tf#/"))
rr.log("/scene_transforms", rr.Transform3D(translation=[-0.016, 0.0, 0.595], 
                                               rotation = rr.RotationAxisAngle(axis=(0, 1, 0), degrees=-90),
                                               child_frame="right_base_link", parent_frame="tf#/"))
# rr.log_file_from_path(urdf_path, static = True)
