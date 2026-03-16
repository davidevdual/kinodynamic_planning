###
### A simple example of hor to achieve torque control for the Panda Franka arm.
### The joints output the torques that they are producing along each axis.
###

import os
import pybullet_data
import math
import time
import pybullet as p
import numpy as np

# function to load the panda robot, with an option to load it in GUI or DIRECT mode
def load_panda(gui=True):
    cid = p.connect(p.GUI if gui else p.DIRECT);
    p.setAdditionalSearchPath(pybullet_data.getDataPath());
    p.setGravity(0,0,-9.81);
    p.setPhysicsEngineParameter(fixedTimeStep=1/240, numSolverIterations=200);
    plane = p.loadURDF("plane.urdf");

    # Load the panda robot, with self-collision enabled (so we can see the effect of dynamics when the robot collides with itself)
    panda = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "franka_panda/panda.urdf"), useFixedBase=True,flags=p.URDF_USE_SELF_COLLISION);
  
    # Disable default motors (so we can use position or torque control manually)
    for j in range(7):
        p.setJointMotorControl2(panda, j, p.VELOCITY_CONTROL, force=0);
        p.enableJointForceTorqueSensor(panda, j, enableSensor=True);

    return panda, cid;

# simple function to interpolate between two joint configurations, and return a list of joint configurations that can be used for motion planning
def interpolate(q_start, q_goal, steps=200):
    return [(1 - t)*q_start + t*q_goal for t in np.linspace(0,1,steps)];

def main():
    panda, cid = load_panda(gui=True);

    # Place the point of view close to the robot
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2]);

    while True:
        # Joints have to achieve torques
        p.setJointMotorControl2(panda, 0, p.TORQUE_CONTROL, force=87);# joint 1
        p.setJointMotorControl2(panda, 1, p.TORQUE_CONTROL, force=87);# joint 2
        p.setJointMotorControl2(panda, 2, p.TORQUE_CONTROL, force=87);# joint 3
        p.setJointMotorControl2(panda, 3, p.TORQUE_CONTROL, force=87);# joint 4
        p.setJointMotorControl2(panda, 4, p.TORQUE_CONTROL, force=10);# joint 5
        p.setJointMotorControl2(panda, 5, p.TORQUE_CONTROL, force=10);# joint 6
        p.setJointMotorControl2(panda, 6, p.TORQUE_CONTROL, force=10);# joint 7

        # Measure the torque produced by the robot at each joint, and print it out
        for j in range(7):
            js = p.getJointState(panda, j);
            Fx, Fy, Fz, Tx, Ty, Tz = js[2];
            print(f"Joint {j}: pos={js[0]:.3f}, vel={js[1]:.3f}, reaction torque (Tx,Ty,Tz)=({Tx:.3f}, {Ty:.3f}, {Tz:.3f})");

        p.stepSimulation();
        time.sleep(1/240);

if __name__ == "__main__":
    main();


    