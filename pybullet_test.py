import pybullet
import pybullet_data
import os
import time 
import math


def main():
    #Connect to the physics environment
    cid = pybullet.connect(pybullet.SHARED_MEMORY)
    if (cid < 0):
        pybullet.connect(pybullet.GUI)

    pybullet.resetSimulation()

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    try:
        os.system("git clone https://github.com/ros-industrial/kuka_experimental.git")
    except:
        print("Kuka URDF is already installed or there was an error.")

    plane = pybullet.loadURDF("plane.urdf")
    robot = pybullet.loadURDF("kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf", [0, 0, 0], useFixedBase=1)  #fixes base to the plane

    pybullet.setGravity(0, 0, -9.81)
    pybullet.setTimeStep(0.0001)
    pybullet.setRealTimeSimulation(0)

    #Get intial joint positions
    [j1_init, j2_init, j3_init, j4_init, j5_init, j6_init] = [j[0] for j in pybullet.getJointStates(robot, range(6))]

    #set up user inputs 
    j1_comm = pybullet.addUserDebugParameter("J1", 0, 2*math.pi, 0)
    j2_comm = pybullet.addUserDebugParameter("J2", 0, 2*math.pi, 0)
    j3_comm = pybullet.addUserDebugParameter("J1", 0, 2*math.pi, 0)
    j4_comm = pybullet.addUserDebugParameter("J4", 0, 2*math.pi, 0)
    j5_comm = pybullet.addUserDebugParameter("J5", 0, 2*math.pi, 0)
    j6_comm = pybullet.addUserDebugParameter("J6", 0, 2*math.pi, 0)


    while True:
        j1 = j1_init + pybullet.readUserDebugParameter(j1_comm)
        j2 = j2_init + pybullet.readUserDebugParameter(j2_comm)
        j3 = j3_init + pybullet.readUserDebugParameter(j3_comm)
        j4 = j4_init + pybullet.readUserDebugParameter(j4_comm)
        j5 = j5_init + pybullet.readUserDebugParameter(j5_comm)
        j6 = j6_init + pybullet.readUserDebugParameter(j6_comm)

        robot_pos = [j1, j2, j3, j4, j5, j6]
        pybullet.setJointMotorControlArray(robot, range(6), pybullet.POSITION_CONTROL, targetPositions=robot_pos)
        pybullet.stepSimulation()


if __name__ == "__main__":
    main()