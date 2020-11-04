import pybullet as p
import pybullet_data
import loader
import time
import struct
import os
import numpy as np
import object_playback


# A sensory board object
class Board(object):
    def __init__(self):
        self.dataset_root = '/Users/liuzeyi/Desktop/sapien/partnet-mobility-v0'
        self.object_ids = [file for file in os.listdir(self.dataset_root) if file.isdigit()]

    # Returns given number of distinctive object ids
    def sample(self, category="switch", sample_num=5):
        # path = os.path.join(self.dataset_root, category)
        return np.random.choice(self.object_ids, sample_num, replace=False)

    def generate_board(self, sample_num=5):
        # Set up the initial environment
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.loadURDF("plane.urdf")

        object_ids = self.sample(sample_num)
        # Generate random positions for objects
        x_index = np.random.choice(np.arange(-sample_num, sample_num), sample_num, replace=False)
        y_index = np.random.choice(np.arange(-sample_num, sample_num), sample_num, replace=False)
        for i, object_id in enumerate(object_ids):
            urdf_path, rotation, offset_z = loader.fetch_mobility_object(object_id)
            object = p.loadURDF(
                urdf_path,
                basePosition=[x_index[i], y_index[i], offset_z],
                baseOrientation=p.getQuaternionFromEuler(rotation),
                useFixedBase=True
            )

            # TODO: Search for valid positions for new objects (heuristics)

        while True:
            p.stepSimulation()


if __name__ == "__main__":
    board = Board()
    board.generate_board()