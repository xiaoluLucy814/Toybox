import os
import numpy as np
import json

def fetch_mobility_object(object_id):
    mobility_path = 'partnet-mobility-v0'
    object_path = os.path.join(mobility_path, object_id)
    urdf_path = os.path.join(object_path, 'mobility_new.urdf')

    with open(os.path.join(object_path, 'bounding_box.json'), 'r') as f:
        bbox = json.load(f)

    # to make the bbox alined with [x, y, z]. Very dirty:-(
    tmp_min = bbox['min'].copy()
    tmp_max = bbox['max'].copy()
    bbox['min'] = [-tmp_max[2], -tmp_max[0], tmp_min[1]]
    bbox['max'] = [-tmp_min[2], -tmp_min[0], tmp_max[1]]

    # rotate np.pi/2 in y-axis. Very dirty:-(
    rotation = [0, np.pi/2, 0]
    offset_z = bbox['max'][0]

    return urdf_path, rotation, offset_z