import yaml

import render_zarr
import render_bagfile_row

with open('config.yaml', 'r') as file:
    loaded_data = yaml.safe_load(file)
    if loaded_data["datatype"] == "zarr":
        render_zarr.render(loaded_data)
    elif loaded_data["datatype"] == "rosbag":
        render_bagfile_row.render(loaded_data)

print("data loaded, enter to quit")
input()