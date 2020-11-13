import pybullet as p
import pybullet_data
import loader
import time
import struct
import os
import csv
import numpy as np

def readLogFile(log_file):
    log = []
    with open(log_file, 'r') as f:
        for row in csv.reader(f):
            log.append(row)
    return log


def replay_states(log_file, object_id):
    """
    Reads states from log file and replays the interactions

    :param log_file: Path to the log file
    :param object_id (str): id of the object in the sapien dataset
    :return: None
    """
    cid = p.connect(p.SHARED_MEMORY)
    if cid < 0:
        p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.resetSimulation()
    p.loadURDF("plane.urdf")

    offset_x, offset_y = 0, 0
    urdf_path, rotation, offset_z = loader.fetch_mobility_object(object_id)
    object = p.loadURDF(
        urdf_path,
        basePosition=[offset_x, offset_y, offset_z],
        baseOrientation=p.getQuaternionFromEuler(rotation),
        useFixedBase=True
    )

    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    log = readLogFile(log_file)

    recordNum = len(log)
    itemNum = len(log[0])
    print('record num:'),
    print(recordNum)
    print('item num:'),
    print(itemNum)

    numJoints = p.getNumJoints(object)
    pointer = 1
    while pointer < recordNum:
        for jointIndex in range(numJoints):
            record = log[pointer + jointIndex]
            position, velocity, joint_motor_torque = float(record[2]), float(record[3]), float(record[10])
            p.setJointMotorControl2(object,
                                    jointIndex,
                                    p.TORQUE_CONTROL,
                                    targetPosition=position,
                                    targetVelocity=velocity,
                                    force=-joint_motor_torque*10)  # increase the force proportionally
        # apply forces on link? need the position on the link to apply force
        p.stepSimulation()
        time.sleep(1. / 10.)
        pointer += numJoints


def log_states(filename, object_id):
    """
    Records manual interactions and write to a log file

    :param filename: Path to the output file
    :param object_id (str): id of the object in the sapien dataset
    :return: None
    """
    cid = p.connect(p.SHARED_MEMORY)
    if cid < 0:
        p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.resetSimulation()
    p.loadURDF("plane.urdf")

    offset_x, offset_y = 0, 0
    urdf_path, rotation, offset_z = loader.fetch_mobility_object(object_id)
    object = p.loadURDF(
        urdf_path,
        basePosition=[offset_x, offset_y, offset_z],
        baseOrientation=p.getQuaternionFromEuler(rotation),
        useFixedBase=True
    )

    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    numJoints = p.getNumJoints(object)
    for i in range(numJoints):
        p.enableJointForceTorqueSensor(object, i, enableSensor=True)

    records = []
    # log records in 100 frames
    for timestamp in range(100):
        p.stepSimulation()
        for i in range(numJoints):
            records.append([timestamp, i, p.getJointState(object, i)])
        time.sleep(1. / 10.)

    dump2file(filename, records)
    p.disconnect()


def dump2file(filename, record):
    """
    Writes to a log file

    :param filename: Path to the output file
    :param record: States recorded during simulation
    :return: None
    """
    f = open(filename, 'w')
    f.write('[timestamp], jointIndex, position, velocity, fx, fy, fz, mx, my, mz, appliedJointMotorTorque\n')
    for timestamp, jointIndex, data in record:
        f.write("[" + str(timestamp) + "], " + str(jointIndex) + ", ")
        for i in range(len(data)):
            if i == 2:
                for ele in data[i]:
                    f.write(str(ele) + ", ")
            elif i == len(data) - 1:
                f.write(str(data[i]) + "\n")
            else:
                f.write(str(data[i]) + ", ")

    f.close()


if __name__ == "__main__":
    # If a log file exists for the given object, load the file and replay states
    # If a log file does not exist, log manual interactions
    object_id = '100968'
    log_file = os.path.join("log", "{}.TXT".format(object_id))
    if os.path.exists(log_file):
        print("LOADING LOG STATES")
        replay_states(log_file, object_id)
    else:
        print("LOGGING STATES")
        log_states(log_file, object_id)