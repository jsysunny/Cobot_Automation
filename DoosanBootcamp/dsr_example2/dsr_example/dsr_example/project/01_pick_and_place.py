import rclpy
import os
import sys

import random
import numpy as np
import time

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 60, 60
ON, OFF = 1, 0

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
        rclpy.init(args=args)

        node = rclpy.create_node('dsr_example_demo_py', namespace=ROBOT_ID)

        DR_init.__dsr__node = node

        try:
                from DSR_ROBOT2 import (print_ext_result, 
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
                                        set_digital_output,
                                        )
                # print_result("Import DSR_ROBOT2 Success!")
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return
        
        # ==================== Set values ====================

        set_velx(30, 20)
        set_accx(60, 40)

        Jhome = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        set_tool("Tool Weight1")
        set_tcp("GripperDA_v1")
        
        # 블럭의 z값 : 272.02, 282.00, 291.92
        # Xa1 = [412.36, 143.43, 310.00, 7.64, -177.23, 7.44]
        # Xa2 = [462.30, 146.58, 310.00, 9.33, -177.33, 9.33]
        # Xa3 = [512.73, 41.70, 310.00, 10.81, -177.69, 10.87]
        
        # Xb1 = [512.49, -54.94, 256.23, 16.25, -176.97, 16.59]
        # Xb2 = [462.35, -105.59, 256.18, 16.04, -176.67, 16.54]
        # Xb3 = [413.05, -155.35, 256.40, 12.99, -176.25, 13.26]
        print('check')
        
        # ==================== A variety of motions ====================

        movej(Jhome, v=VELOCITY, a=ACC)

        # movel(Xa1, v=VELOCITY, a=ACC)
        task_compliance_ctrl(stx=[1000, 1000, 1000, 100, 100, 100])
        set_desired_force(fd=[0, 0, -50, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        
        # while True:
                # tool_force = get_tool_force()[2]
                # print(f'tool_force: {tool_force:.3f}')
        time.sleep(5)

        release_force()
        release_compliance_ctrl()

        movej(Jhome, v=VELOCITY, a=ACC)

        rclpy.shutdown()

if __name__ == "__main__":
        main()