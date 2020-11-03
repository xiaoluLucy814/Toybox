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
        offset_x, offset_y = 0, 0

        for object_id in self.sample(sample_num):

            urdf_path, rotation, offset_z = loader.fetch_mobility_object(object_id)
            object = p.loadURDF(
                urdf_path,
                basePosition=[offset_x, offset_y, offset_z],
                baseOrientation=p.getQuaternionFromEuler(rotation),
                useFixedBase=True
            )

            # TODO: consider the layout of the board
            offset_x = offset_x + 2

        while True:
            p.stepSimulation()


if __name__ == "__main__":
    board = Board()
    board.generate_board()