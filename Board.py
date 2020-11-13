import pybullet as p
import pybullet_data
import loader
import time
import os
import numpy as np
import json


class Board(object):
    """
    A sensory board with sampled objects from the sapien databset
    """
    def __init__(self, board_length=10, board_width=10, board_pos=(0, 0)):
        self.dataset_root = '/Users/liuzeyi/Desktop/sapien/partnet-mobility-v0'
        self.board_length = board_length
        self.board_width = board_width
        self.board_pos = board_pos
        self.occupied_area = []
        self.board_boundary = np.array([
            [board_pos[0] - board_length / 2, board_pos[0] + board_length / 2],  # 2x2 rows: x, y cols:min, max
            [board_pos[0] - board_width / 2, board_pos[0] + board_width / 2],
        ])

    def sample_from_category(self, category="switch", sample_num=5):
        """

        Samples given number of objects from a specific category

        :param category: object category, e.g. switch, door, ...
        :param sample_num: number of objects to sample from the dataset
        :return: {sample_num} number of distinctive object ids
        """
        # read json file for ids of objects from the same category
        object_ids = [file for file in os.listdir(self.dataset_root) if file.isdigit()]
        return np.random.choice(object_ids, sample_num, replace=False)

    def generate_board(self, sample_num=5):
        """
        Generates board with given number of objects
        - The objects are guaranteed to be non-overlap
        TODO: Make sure we will get enough information of the board from the camera view

        :param sample_num: number of sampled objects on the board
        :return: None
        """
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        p.loadURDF("plane.urdf")

        object_ids = self.sample_from_category(sample_num=sample_num)
        objects = []
        for i, object_id in enumerate(object_ids):
            urdf_path, rotation, offset_z = loader.fetch_mobility_object(object_id)
            offset_x, offset_y = self.find_position(object_id)
            # If no valid position is found, offset_x = None, offset_y = None
            if not (offset_x and offset_y):
                print("Fail to find position for the current object, skip to the next object")
                continue
            object = p.loadURDF(
                urdf_path,
                basePosition=[offset_x, offset_y, offset_z],
                baseOrientation=p.getQuaternionFromEuler(rotation),
                useFixedBase=True
            )
            objects.append(object)

        # p.stepSimulation()
        # assert(not self.sanity_check_overlap(objects))

        while True:
            p.stepSimulation()

    def get_bounding_box_in_board_coordinates(self, object_id, object_pos):
        """
        Gets the bounding box of the given object in the board coordinates

        :param object_id (str): id of the object in the sapien dataset
        :param object_pos (Numpy array [1x2]): base position of the object [x, y]
        :return: bounding box of the object (Numpy array [2x2]): 2x2 rows: x, y cols:min, max
        """
        bbox = np.zeros(shape=(2, 2))
        f = open(os.path.join(self.dataset_root, object_id, "bounding_box.json"))
        data = json.load(f)
        lower_bound, upper_bound = np.array(data['min'])[:2], np.array(data['max'])[:2]
        bbox[0, 0] = object_pos[0] + lower_bound[0]
        bbox[0, 1] = object_pos[0] + upper_bound[0]
        bbox[1, 0] = object_pos[1] + lower_bound[1]
        bbox[1, 1] = object_pos[1] + upper_bound[1]
        return bbox

    def find_position(self, object_id, MAX_ITERATIONS=10):
        """
        Searches for valid position to place the given object

        :param object_id (str): id of the object in the sapien dataset
        :param MAX_ITERATIONS: max number of trials
        :return: x and y position of the object on the board;
                 if no position is found, return None, None
        """
        # Generate random positions for objects
        offset_x, offset_y, bbox = 0, 0, None
        found = False
        for i in range(MAX_ITERATIONS):
            print(object_id, i)
            offset_x = np.random.uniform(self.board_boundary[0, 0], self.board_boundary[0, 1], 1)[0]
            offset_y = np.random.uniform(self.board_boundary[1, 0], self.board_boundary[1, 1], 1)[0]
            bbox = self.get_bounding_box_in_board_coordinates(object_id, np.array([offset_x, offset_y]))

            if not self.check_overlap(bbox) and not self.check_off_board(bbox):
                found = True
                break

        if found:
            self.occupied_area.append(bbox)
            return offset_x, offset_y
        if not found:
            return None, None

    def check_overlap(self, bbox):
        """
        Checks if the placement of a new object overlaps with existing objects
        ref: https://www.geeksforgeeks.org/find-two-rectangles-overlap/

        :param bbox (Numpy array [2x2]): bounding box of an object
        :return: True if there is overlap; false if there isn't
        """
        for area in self.occupied_area:
            # If one rectangle is on left side of other
            x_not_overlap = (bbox[0, 0] >= area[0, 1] or area[0, 0] >= bbox[0, 1])
            # If one rectangle is above other
            y_not_overlap = (bbox[1, 1] <= area[1, 0] or area[1, 1] <= bbox[1, 0])
            if not x_not_overlap and not y_not_overlap:
                return True
        return False

    def check_off_board(self, bbox):
        """
        Checks if the object exceeds the bound of the board

        :param bbox (Numpy array [2x2]): bounding box of an object
        :return: True if object is out of bound; false if object is within the bound
        """
        if bbox[0, 0] < self.board_boundary[0, 0] or bbox[0, 1] > self.board_boundary[0, 1]:
            return True
        if bbox[1, 0] < self.board_boundary[1, 0] or bbox[1, 1] > self.board_boundary[1, 1]:
            return True
        return False

    def sanity_check_overlap(self, objects):
        """
        A sanity check for overlap during simulation.

        :param objects: bodyIds of all objects in the pybullet simulation environment
        :return: True if there is overlap; false if there isn't
        """
        for i in range(len(objects)):
            for j in range(i+1, len(objects)):
                object_A, object_B = objects[i], objects[j]
                overlap_points = p.getContactPoints(object_A, object_B)
                if overlap_points is not None and len(overlap_points) != 0:
                    return True
        return False


if __name__ == "__main__":
    board = Board()
    board.generate_board()