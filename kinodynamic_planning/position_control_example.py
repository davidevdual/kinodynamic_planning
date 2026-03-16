###
### A simple example of hor to achieve position control for the Panda Franka arm.
### The joints output the torques that they are producing along each axis.
### Position control alone generates a significant amount of torque that violates constraints.
###

import os
import pybullet_data
import math
import time
import pybullet as p
import numpy as np

MAX_FORCE = 400.0; # Very high torque limit per joint (N.m) in simulation, to see the effect of position control without torque limits
TAU_BEST_CASE = 70.0; # Nominal torque range (+/- 10 Nm) - commonly quoted
TAU_NOMINAL = 10.0; # "Best-case" sensing range on some axes (not a safe limit)

# function to convert from degrees to radians (for convenience)
# @param degrees: angle in degrees
# @return: angle in radians
def deg2rad(degrees):
    return degrees * math.pi / 180.0;

# function to convert from radians to degrees (for convenience)
# @param radians: angle in radians
# @return: angle in degrees
def rad2deg(radians):
    return radians * 180.0 / math.pi;

# function to load the panda robot, with an option to load it in GUI or DIRECT mode
def load_panda(gui=True):
    cid = p.connect(p.GUI if gui else p.DIRECT);
    p.setAdditionalSearchPath(pybullet_data.getDataPath());
    p.setGravity(0,0,-9.81);
    p.setPhysicsEngineParameter(fixedTimeStep=1/240, numSolverIterations=200);

    # Load the panda robot, with self-collision enabled (so we can see the effect of dynamics when the robot collides with itself)
    pandaUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "franka_panda/panda.urdf"),useFixedBase=True,flags=p.URDF_USE_SELF_COLLISION);
    tableUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),basePosition=[0.5,0,-0.65]);

    return pandaUid, cid;

# simple function to interpolate between two joint configurations, and return a list of joint configurations that can be used for motion planning
def interpolate(q_start, q_goal, steps=200):
    return [(1 - t)*q_start + t*q_goal for t in np.linspace(0,1,steps)];

def main():
    panda, cid = load_panda(gui=True);

    # Place the point of view close to the robot
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2]);
    
    # Wait for a few seconds to let the user see the robot before it starts moving
    time.sleep(5);

    while True:
        # Joints have to achieve positions without torque limits
        #p.setJointMotorControl2(panda, 0, p.POSITION_CONTROL, targetPosition = deg2rad(0));# joint 1
        #p.setJointMotorControl2(panda, 1, p.POSITION_CONTROL, targetPosition = deg2rad(0));# joint 2
        #p.setJointMotorControl2(panda, 2, p.POSITION_CONTROL, targetPosition = deg2rad(0));# joint 3
        #p.setJointMotorControl2(panda, 3, p.POSITION_CONTROL, targetPosition = deg2rad(0));# joint 4
        #p.setJointMotorControl2(panda, 4, p.POSITION_CONTROL, targetPosition = deg2rad(0));# joint 5
        p.setJointMotorControl2(panda, 5, p.POSITION_CONTROL, targetPosition = deg2rad(45));# joint 6
        #p.setJointMotorControl2(panda, 6, p.POSITION_CONTROL, targetPosition = deg2rad(0));# joint 7
        
        # Measure the torque produced by the robot at each joint, and print it out
        for j in range(7):
            js = p.getJointState(panda, j);
            T = js[3];# Torque applied by the joint motor to achieve the position control target
            print(f"Joint {j}: pos={rad2deg(js[0]):.3f}, vel={js[1]:.3f}, applied joint torque ({T:.3f})");

        p.stepSimulation();
        time.sleep(1/240);

if __name__ == "__main__":
    main();


    