#!/usr/bin/env python

from bullet_arm_1dof import BulletArm1Dof
from robot_arm_1dof import RobotArm1Dof

import pybullet

if __name__ == "__main__":
    r_bullet = BulletArm1Dof()
    r_bullet.InitPybullet(pybullet.GUI)

    r_arm = r_bullet.LoadRobot()

    while True:
        pass

else:
    r_bullet = BulletArm1Dof()
    r_bullet.InitPybullet(bullet_connect=pybullet.GUI)

    r_arm = r_bullet.LoadRobot()