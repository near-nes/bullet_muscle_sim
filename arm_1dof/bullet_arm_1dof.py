import os

import arm_1dof.robot_arm_1dof
import arm_1dof.robot_arm_muscle_1dof
import pybullet
import numpy as np


class BulletArm1Dof:
    """Class to initialize pybullet server and load robot"""
    SDF_MODELS_SUBDIR="arm_1dof"
    SDF_SEARCH_PATH="/".join([os.environ.get("SDF_MODELS_DIR", "."), SDF_MODELS_SUBDIR])
    SDF_MODEL_NAME="demo_human_col"
    SDF_MODEL_FILENAME="/".join([SDF_MODEL_NAME, "skeleton.sdf"])
    URDF_MODEL_FILENAME="/".join([SDF_MODEL_NAME, "skeleton.urdf"])
    PLANE_MODEL_FILENAME="/".join([SDF_MODEL_NAME, "plane.urdf"])

    def __init__(self) -> None:
        self._server_id = -1
        self._skel_arm = None
        self._skel_arm_id = -1
        self._skel_muscles = None

    @property
    def server_id(self):
        """Pybullet server ID"""
        return self._server_id

    def InitPybullet(self, g=[0., 0., 0.], bullet_connect=pybullet.DIRECT, sdf_search_path=SDF_SEARCH_PATH) -> None:
        """Start bullet server and set sdf search path"""
        # Connect to pybullet
        self._server_id = pybullet.connect(bullet_connect)

        pybullet.setGravity(g[0], g[1], g[2], physicsClientId=self._server_id)
        pybullet.setAdditionalSearchPath(path=sdf_search_path, physicsClientId=self._server_id)

        self._timestep = pybullet.getPhysicsEngineParameters(physicsClientId=self._server_id)['fixedTimeStep']

    def LoadPlane(self, robot_sdf_filename=PLANE_MODEL_FILENAME):
        """Load plane from sdf file"""
        plane_id = pybullet.loadURDF(fileName=robot_sdf_filename, useFixedBase=True, physicsClientId=self._server_id)
        # Define new position (x, y, z)
        new_position = [0, 10, 10]  # Change as needed

        # Create rotation quaternion for 90 degrees around Z-axis
        # For other axes: X-axis = [1,0,0], Y-axis = [0,1,0], Z-axis = [0,0,1]
        rotation_quaternion = pybullet.getQuaternionFromEuler(
            [np.pi / 2, 0, 0]
        )  # 90 deg around Z

        # Apply the transformation
        pybullet.resetBasePositionAndOrientation(
            plane_id, new_position, rotation_quaternion, physicsClientId=self._server_id
        )

        self._plane = plane_id
        return self._plane

    def LoadRobot(self, robot_sdf_filename=URDF_MODEL_FILENAME) -> arm_1dof.robot_arm_1dof.RobotArm1Dof:
        """Load robot from sdf file"""
        self._skel_arm_id = pybullet.loadURDF(fileName=robot_sdf_filename, useFixedBase=True, physicsClientId=self._server_id)

        self._skel_arm = arm_1dof.robot_arm_1dof.RobotArm1Dof(bullet_ctrl=self, bullet_body_id=self._skel_arm_id)

        return self._skel_arm

    def LoadMuscleRobot(self, robot_sdf_filename=URDF_MODEL_FILENAME) -> arm_1dof.robot_arm_muscle_1dof.RobotArmMuscle1Dof:
        """Load robot from sdf file, add muscles"""
        self._skel_arm_id = pybullet.loadURDF(fileName=robot_sdf_filename, useFixedBase=True, physicsClientId=self._server_id)

        self._skel_arm = arm_1dof.robot_arm_muscle_1dof.RobotArmMuscle1Dof(bullet_ctrl=self, bullet_body_id=self._skel_arm_id)
        self._skel_muscles = self._skel_arm.muscles

        return self._skel_arm

    def LoadTarget(self, target_position, target_color=[1, 0, 0, 1]):
        ball_shape = pybullet.createVisualShape(
            shapeType=pybullet.GEOM_SPHERE,
            radius=0.02,
            rgbaColor=target_color,
            physicsClientId=self._server_id,
        )
        ball_collision = pybullet.createCollisionShape(
            shapeType=pybullet.GEOM_SPHERE, radius=0.02, physicsClientId=self._server_id
        )
        ball = pybullet.createMultiBody(
            baseMass=0,
            baseInertialFramePosition=[0, 0, 0],
            baseCollisionShapeIndex=ball_collision,
            baseVisualShapeIndex=ball_shape,
            basePosition=target_position,
            physicsClientId=self._server_id,
        )
        return ball
        # ball_const = pybullet.createConstraint(
        #     self._skel_arm,
        #     4,
        #     ball,
        #     -1,
        #     pybullet.JOINT_FIXED,
        #     [0, 0, 0],
        #     pybullet.getJointState(self._skel_arm, 4)[:2],
        #     [0, 0, 0],
        #     [0, 0, 0],
        #     [0, 0, 0],
        #     physicsClientId=self._server_id,
        # )

    def Simulate(self, sim_time) -> None:
        t = 0
        while t < sim_time:
            pybullet.stepSimulation(physicsClientId=self._server_id)

            if self._skel_muscles:
                # Update muscle joint states
                self._skel_arm.UpdateStats()
                self._skel_muscles.SetJointPos(JOINT_INDEX(0), self._skel_arm.JointPos(0))
                self._skel_muscles.SetJointPos(JOINT_INDEX(1), self._skel_arm.JointPos(1))
                self._skel_muscles.SetJointVel(JOINT_INDEX(0), self._skel_arm.JointVel(0))
                self._skel_muscles.SetJointVel(JOINT_INDEX(1), self._skel_arm.JointVel(1))

                self._skel_muscles.Integrate(self._timestep)

                # Apply muscle torques to robot
                torques = [ self._skel_muscles.GetTorque(JOINT_INDEX(0)), self._skel_muscles.GetTorque(JOINT_INDEX(1))]
                self._skel_arm.SetJointTorques(joint_ids=[0,1], torques=torques)

            t += self._timestep
