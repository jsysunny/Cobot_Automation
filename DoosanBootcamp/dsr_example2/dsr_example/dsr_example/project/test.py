import rclpy
import os
import sys

import random
import numpy as np

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

import DR_init
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
        rclpy.init(args=args)

        node = rclpy.create_node('dsr_example_demo_py', namespace=ROBOT_ID)

        DR_init.__dsr__node = node

        try:
                from DSR_ROBOT2 import print_ext_result, movej, movel, movec, move_periodic, move_spiral, set_velx, set_accx, DR_BASE, DR_TOOL, DR_AXIS_X, DR_MV_MOD_ABS, get_current_posj
                # print_result("Import DSR_ROBOT2 Success!")
        except ImportError as e:
                print(f"Error importing DSR_ROBOT2 : {e}")
                return
        #################
        ### Set values ##
        set_velx(30, 20)
        set_accx(60, 40)

        Jhome = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        # J1 = [180.0, -45.0, 90.0, 0.0, -45.0, 180.0]
        # J2 = [180.0, 45.0, -90.0, 0.0, 45.0, 0.0]

        j1, j2, j3, j4, j5, j6 = 0.0, 0.0, 90.0, 0.0, 90.0, 0.0
        
        ###########################
        ### A variety of motions ##

        movej(Jhome, v=90, a=90)
        while rclpy.ok():
                j1 = np.clip(j1 + float(random.randint(-20, 20)), -180.0, 180.0)
                j2 = np.clip(j2 + float(random.randint(-20, 20)), -70.0, 70.0)
                j3 = np.clip(j3 + float(random.randint(-20, 20)), -100.0, 100.0)
                j4 = np.clip(j4 + float(random.randint(-20, 20)), -180.0, 180.0)
                j5 = np.clip(j5 + float(random.randint(-20, 20)), -150.0, 150.0)
                j6 = np.clip(j6 + float(random.randint(-20, 20)), -180.0, 180.0)
                
                pos = [j1, j2, j3, j4, j5, j6]
                print(f"pos = {get_current_posj()}")

                movej(pos, v=60, a=60)
        
        print('good bye!')
        rclpy.shutdown()

if __name__ == "__main__":
        main()