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
            records.append([timestamp, i, p.getJointState(object, i) + p.getLinkState(object, i)[0]])
        time.sleep(1. / 10.)

    dump2file(filename, records)
    p.disconnect()


def dump2file(filename, record):
    f = open(filename, 'w')
    f.write('[timestamp], jointIndex, position, velocity, fx, fy, fz, mx, my, mz, appliedJointMotorTorque, lx, ly, lz\n')
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
    object_id = '100968'
    log_file = os.path.join("log", "{}.TXT".format(object_id))
    if os.path.exists(log_file):
        print("LOADING LOG STATES")
        replay_states(log_file, object_id)
    else:
        print("LOGGING STATES")
        log_states(log_file, object_id)


'''
# function to read and parse a given log file
def readLogFile(filename, verbose=True):
    f = open(filename, 'rb')

    print('Opened'),
    print(filename)

    keys = f.readline().decode('utf8').rstrip('\n').split(',')
    fmt = f.readline().decode('utf8').rstrip('\n')

    # The byte number of one record
    sz = struct.calcsize(fmt)
    # The type number of one record
    ncols = len(fmt)

    if verbose:
        print('Keys:'),
        print(keys)
        print('Format:'),
        print(fmt)
        print('Size:'),
        print(sz)
        print('Columns:'),
        print(ncols)

    # Read data
    wholeFile = f.read()
    # split by alignment word
    chunks = wholeFile.split(b'\xaa\xbb')
    log = list()
    for chunk in chunks:
        if len(chunk) == sz:
            values = struct.unpack(fmt, chunk)
            record = list()
            for i in range(ncols):
                record.append(values[i])
            log.append(record)

    return log


# write state records into a log file
def log_states(log_file, object_id):
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

    logId = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, log_file, [object])

    # Record logging for 100 frames
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1. / 10.)

    p.stopStateLogging(logId)
    p.disconnect()
'''