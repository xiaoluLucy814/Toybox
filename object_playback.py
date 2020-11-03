import pybullet as p
import pybullet_data
import loader
import time
import struct
import os

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


# replay states recorded in a given log file
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

    for record in log:
        pos = [record[3], record[4], record[5]]
        orn = [record[6], record[7], record[8], record[9]]
        numJoints = p.getNumJoints(object)
        targetPositionsJoints = [record[29]]
        for i in range(numJoints):
            jointInfo = p.getJointInfo(object, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                targetPositionsJoints.append(record[qIndex - 7 + 17])

        print(targetPositionsJoints)
        p.setJointMotorControlArray(object,
                                    range(numJoints),
                                    p.POSITION_CONTROL,
                                    targetPositions=targetPositionsJoints)

        p.stepSimulation()
        time.sleep(1. / 20.)


if __name__ == "__main__":
    object_id = '100968'
    log_file = os.path.join("log", "{}.TXT".format(object_id))
    if os.path.exists(log_file):
        print("LOADING LOG STATES")
        replay_states(log_file, object_id)
    else:
        print("LOGGING STATES")
        log_states(log_file, object_id)
