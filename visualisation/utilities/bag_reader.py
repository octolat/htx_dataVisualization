from logging import raiseExceptions
from pathlib import Path
import struct

from rosbags.rosbag2 import Reader
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

import time
import numpy as np

bagpath = Path(__file__).parent / "resources" / "rawbags/B-KING37-RQ2F85-L515-R-L_TBLTP_P_BLUBTL-LOC4_2026-02-26-18-11-11_S/B-KING37-RQ2F85-L515-R-L_TBLTP_P_BLUBTL-LOC4_2026-02-26-18-11-11_0.db3"
typestore = get_typestore(Stores.ROS2_HUMBLE)

# # Create reader instance and open for reading.
# with AnyReader([bagpath], default_typestore=typestore) as reader:
#     count = 0
#     for connection, timestamp, rawdata in reader.messages(connections=reader.connections):
#         msg = reader.deserialize(rawdata, connection.msgtype)
#         print(connection.topic)
#         count += 1
#         if count > 50:
#             break

# # Create reader instance and open for reading.
# with AnyReader([bagpath], default_typestore=typestore) as reader:
#     connections = [x for x in reader.connections if x.topic == '/right_arm/arm_feedback']
#     count = 0
#     for connection, timestamp, rawdata in reader.messages(connections=connections):
#         msg = reader.deserialize(rawdata, connection.msgtype)
#         print(f"{time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(timestamp/1e9))}, {msg.position[6]}")
#         # if count % 10 == 0: print(msg.position)
#         # print(msg.name[7])
#         count += 1


# def convertPointCloud(msg):
#     TIME = time.time()
#     # constants
#     SIZE = {7: 4} #see fields datatype enums

#     #parse fields array
#     fields = {}
#     for field in msg.fields:
#         contents = {
#             "offset": field.offset,
#             "datatype": field.datatype,
#         }
#         fields[field.name] = contents

#     #parse endian format
#     if not msg.is_bigendian:
#         format = "<"
#     else:
#         format = ">"
#     #check if data type is unsupported (currently only float32)
#     for field in fields.values():
#         if field["datatype"] != 7:
#             raise ValueError("pointcloud datatype must be float32")
#     format += "f"
    
#     points = np.empty((msg.height, msg.width, 3), dtype=np.float32)
#     for row_idx in range(msg.height):
#         for point_idx in range(msg.width):
#             start = row_idx * msg.row_step + point_idx * msg.point_step 
#             for axis_idx, axis in enumerate(['x', 'y', 'z']):
#                 section = msg.data[start + fields[axis]["offset"]: start + fields[axis]["offset"] + SIZE[fields[axis]["datatype"]]]
#                 points[row_idx][point_idx][axis_idx] = struct.unpack(format, bytes(section))[0]
    
#     print(time.time() - TIME)
#     return points

def convertPointCloud(msg):
    # TIME = time.time()
    # Parse all raw bytes into a numpy array at once
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    # Determine endianness
    endian = '>' if msg.is_bigendian else '<'

    # Parse fields to get offsets
    field_map = {f.name: f.offset for f in msg.fields}

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

    # Stack and reshape to (height, width, 3)
    points = np.stack([flat_points['x'], flat_points['y'], flat_points['z']], axis=-1)
    points = points.reshape(msg.height, msg.width, 3).astype(np.float32)

    # print(time.time() - TIME)
    return points


with Reader(bagpath) as reader:
    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        print(connection.topic)
        timestamp = np.datetime64(timestamp, "ns")
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        # if connection.topic == '/camera/color/image_raw':
        #     image = np.array(msg.data).reshape(msg.height, msg.width, 3)
        #     # print(f"{image}, {timestamp}")
        #     print(msg)
        #     break
        # elif connection.topic == '/camera/depth/color/points':
        #     points = convertPointCloud(msg)[0] #assumes only 1 row basically
        #     # print(points)
        #     break
        if connection.topic == '/right_arm/arm_feedback':
            for i in range(7):
                #like if its non infi joint like map it back
                pos = msg.position[i]
                if int(msg.name[i][-1]) % 2 == 0: #TODO: super breakable
                    if pos > 3.14: pos = 3.14 - pos
                print(f"{msg.name[i]}, {pos/3.14*180}")

