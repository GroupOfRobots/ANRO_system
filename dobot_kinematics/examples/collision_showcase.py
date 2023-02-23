import time
import numpy as np
import pybullet as pyb
import pybullet_data
import os.path as path
import sys
from math import radians, degrees
import os
from dobot_kinematics.collision_utils import NamedCollisionObject, CollisionDetector


TIMESTEP = 1.0 / 60

global model_config


def load_environment(client_id):
    pyb.setTimeStep(TIMESTEP, physicsClientId=client_id)
    pyb.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=client_id
    )


    #Dobot Magician robot arm
    robot_model_path = path.abspath(path.join(__file__ ,"../../..")) + "/dobot_description/meshes/collision/urdf_models/suction_cup.urdf"
    global model_config
    prefix, model_config = os.path.split(robot_model_path)
    dobot_magician_id = pyb.loadURDF(
        robot_model_path,
        [0, 0, 0],
        useFixedBase=True,
        physicsClientId=client_id,
    )



    #TODO urdf ground plane
    # ground plane
    ground_id = pyb.loadURDF(
        "plane.urdf", [0, 0, -0.131305], useFixedBase=True, physicsClientId=client_id
    )


    # store body indices in a dict with more convenient key names
    bodies = {
        "robot": dobot_magician_id,
        "ground": ground_id,
    }

    for id in bodies.values():
        if id <= -1 :
            print("Could not load URDF model")
            sys.exit(1)

    return bodies


def create_robot_debug_params(robot_id, gui_id):
    """Create debug params to set the robot joint positions from the GUI."""
    params = {}
    params['magician_joint_1'] = pyb.addUserDebugParameter('magician_joint_1', 
                                                        rangeMin = -125, 
                                                        rangeMax = 125, 
                                                        startValue=0, 
                                                        physicsClientId=gui_id)

    params['magician_joint_2'] = pyb.addUserDebugParameter('magician_joint_2', 
                                                        rangeMin = -5, 
                                                        rangeMax = 90, 
                                                        startValue=0, 
                                                        physicsClientId=gui_id)
   
    params['magician_joint_3'] = pyb.addUserDebugParameter('magician_joint_3', 
                                                        rangeMin = -15, 
                                                        rangeMax = 70, 
                                                        startValue=0, 
                                                        physicsClientId=gui_id)
    return params


def read_robot_configuration(robot_id, robot_params, gui_id):
    """Read robot configuration from the GUI."""
    q = np.zeros(4)
    q[0] = radians(pyb.readUserDebugParameter(robot_params['magician_joint_1'], physicsClientId=gui_id))
    q[1] = radians(pyb.readUserDebugParameter(robot_params['magician_joint_2'], physicsClientId=gui_id))
    q[2] = radians(pyb.readUserDebugParameter(robot_params['magician_joint_3'], physicsClientId=gui_id)) - q[1]
    q[3] = -(q[1]+q[2])

    return q


def update_robot_configuration(robot_id, q, gui_id):
    """Set the robot configuration."""
    for i in range(4):
        pyb.resetJointState(
            robot_id, i+1, q[i], physicsClientId = gui_id
        )
    
def main():

    # main simulation server, with a GUI
    gui_id = pyb.connect(pyb.GUI, options="--opengl3")
    # simulation server only used for collision detection
    col_id = pyb.connect(pyb.DIRECT, options="--opengl3")

    if gui_id == -1 or col_id == -1:
        print("Not connected")
        sys.exit(1)

    # configuration of debug visualizer
    #pyb.configureDebugVisualizer(pyb.COV_ENABLE_GUI, 0)
    pyb.configureDebugVisualizer(pyb.COV_ENABLE_SHADOWS, 0)
    pyb.configureDebugVisualizer(pyb.COV_ENABLE_WIREFRAME, 0)
    pyb.configureDebugVisualizer(pyb.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    pyb.configureDebugVisualizer(pyb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    pyb.configureDebugVisualizer(pyb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    # add bodies to both of the environments
    bodies = load_environment(gui_id)
    collision_bodies = load_environment(col_id)

    robot_id = bodies["robot"]

    robot_params = create_robot_debug_params(robot_id, gui_id)

    # define bodies (and links) to use for shortest distance computations and
    # collision checking
    # ground = NamedCollisionObject("ground")
    link1 = NamedCollisionObject("robot", "magician_link_1")
    link7 = NamedCollisionObject("robot", "magician_link_4")  # last link
    if model_config != "3DOF.urdf":
        link7_2 = NamedCollisionObject("robot", "end_effector_part")  # last link
    base = NamedCollisionObject("robot", "magician_base_link")
    ground = NamedCollisionObject("ground")

    if model_config == "3DOF.urdf":
        col_detector = CollisionDetector(
            col_id,
            collision_bodies,
            # [(link7, ground), (link7, cube1), (link7, cube2), (link7, cube3), (link7, base)],
            [(link7, base), (link7, ground), (link7, link1)],
        )
    else:
        col_detector = CollisionDetector(
            col_id,
            collision_bodies,
            # [(link7, ground), (link7, cube1), (link7, cube2), (link7, cube3), (link7, base)],
            [(link7, base), (link7, ground), (link7, link1), (link7_2, base), (link7_2, ground), (link7_2, link1)],
        )

    robot_id = bodies["robot"]

    while True:
        q = read_robot_configuration(robot_id, robot_params, gui_id)

        # compute shortest distances for user-selected configuration
        d = col_detector.compute_distances(q)
        in_col = col_detector.in_collision(
            q, margin = 0.0
        )

        # move to the requested configuration if it is not in collision,
        # otherwise display a warning
        # the key is that we can check collisions using the separate physics
        # client, so we don't have to set the robot to a configuration in the
        # main GUI sim to check if that configuration is in collision
        if not in_col:
            update_robot_configuration(robot_id, q, gui_id=gui_id)
        else:
            update_robot_configuration(robot_id, q, gui_id=gui_id)
            pyb.addUserDebugText(
                "Avoiding collision",
                textPosition=[0, 0, 1.5],
                textColorRGB=[1, 0, 0],
                textSize=2,
                lifeTime=0.2,
            )

        print(f"Distance to obstacles = {d}")

        names = col_detector.print_collision_pairs(d)
        print("Collision pairs")
        print(*names, sep = "\n")

        pyb.stepSimulation(physicsClientId=gui_id)
        time.sleep(TIMESTEP)


if __name__ == "__main__":
    main()
