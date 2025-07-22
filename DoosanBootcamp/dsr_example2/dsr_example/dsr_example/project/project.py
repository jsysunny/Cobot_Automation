import rclpy
import os
import sys

import time
import numpy as np
from rclpy.executors import MultiThreadedExecutor
import threading

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 20, 20
COLLISION_FORCE_THRESHOLD = 10.0

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

collision_detected = False

def main(args=None):
    """
    메인 함수: ROS 노드 초기화 및 이동 경로 실행
    """
    rclpy.init(args=args)
    node = rclpy.create_node('dsr_control_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        print_ext_result,
        movej,
        movel,
        movec,
        move_periodic,
        move_spiral,
        set_velx,
        set_accx,
        DR_BASE,
        DR_TOOL,
        DR_AXIS_X,
        DR_MV_MOD_ABS,
        get_current_posj,
        task_compliance_ctrl,
        set_desired_force,
        get_tool_force,
        release_compliance_ctrl,
        release_force,
        DR_FC_MOD_REL,
        set_tool,
        set_tcp,
        DR_HOLD,
        DR_MV_MOD_REL
    )

    ## set
    set_velx(30, 20)
    set_accx(60, 40)

    set_tool("Tool Weight1")
    set_tcp("GripperDA_v1")

    ## joing, pose set
    Jhome = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
    Pstart = [162.30, 305.92, 100.00, 53.48, -179.14, 53.20]
    test = [60.86, -0.43, 105.36, 0.10, 65.91, 60.59]
    

#     home = [56.11, 20.33, 72.02, 2.54, 88.79, -85.17]
#     right_down = [42.18, 73.92, 35.88, -1.36, 63.57, -85.17]
#     right_up = [44.98, 74.03, 8.03, -5.05, 68.26, -85.17]
#     left_down = [71.43, 61.93, 57.67, 1.35, 55.24, -85.17]
#     left_up = [80.15, 60.74, 29.14, 1.79, 82.58, -85.17]

    ## motion
    movej(Jhome, v=VELOCITY, a=ACC)
    movej(test, v=VELOCITY, a=ACC)

    movel([400.0, 0, 0, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_MV_MOD_REL)
    

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()