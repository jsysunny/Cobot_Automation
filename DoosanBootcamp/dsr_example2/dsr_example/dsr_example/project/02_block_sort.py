import rclpy
import os
import sys

import random
import numpy as np
import time

import DR_init

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
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
                            get_current_posx,
                            set_digital_output,
                            )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

set_tool("Tool Weight1")
set_tcp("GripperDA_v1")

def grip():
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(0.3)

def release():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(0.3)

def main(args=None):
        # ==================== Set values ====================

        set_velx(30, 20)
        set_accx(60, 40)

        ON, OFF = 1, 0
        VELOCITY, ACC = 80, 80
        FORCE_THRESHOLD = 15

        Jhome = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 블럭의 z값 : 44.16 54.18 64.05

        Xa1 = [351.21, 144.47, 80.00, 11.15, -179.47, 10.89] # 탐색 높이 80으로 맞추기
        Xa2 = [351.84, 93.19, 80.13, 46.79, -179.18, 46.83] #
        Xa3 = [349.29, 41.86, 79.98, 175.19, 179.67, 175.22] #
        Xa4 = [402.08, 143.39, 80.00, 72.39, -179.85, 72.31] #
        Xa5 = [401.02, 92.01, 80.00, 39.63, -179.23, 39.53] #
        Xa6 = [401.73, 41.34, 80.00, 48.91, -178.89, 49.06] # 
        Xa7 = [452.83, 143.03, 80.00, 38.20, -178.40, 37.92] #
        Xa8 = [451.78, 93.07, 80.00, 38.51, -177.97, 38.62] #
        Xa9 = [452.67, 41.90, 80.00, 38.07, -177.74, 38.56] #

        Xa_list = [Xa1, Xa2, Xa3, Xa4, Xa5, Xa6, Xa7, Xa8, Xa9]
        
        Xb1_up = [347.86, -59.05, 135.00, 27.57, -179.13, 27.80] # z위치는 135으로 맞추기
        Xb2_up = [347.89, -109.81, 135.00, 37.49, -178.75, 38.42] #
        Xb3_up = [347.33, -159.67, 135.31, 174.26, 179.42, 173.91] #
        Xb4_up = [400.34, -57.14, 135.00, 11.48, -178.21, 11.53] # 
        Xb5_up = [400.41, -108.20, 134.84, 12.16, -177.91, 12.32] #
        Xb6_up = [396.76, -159.65, 135.00, 65.22, -179.62, 65.66] #
        Xb7_up = [449.28, -57.96, 135.00, 14.71, -178.60, 15.54] #
        Xb8_up = [451.32, -108.46, 135.00, 14.46, -178.20, 15.18] #
        Xb9_up = [452.32, -159.10, 135.00, 12.14, -177.91, 13.02] #

        # 칸 비었는지 확인하는 리스트
        Xb_check_list = [False, False, False, False, False, False, False, False, False]

        print('check')
        
        # ==================== A variety of motions ====================

        ## init position
        release()
        grip()
        movej(Jhome, v=VELOCITY, a=ACC)

        for Xa in Xa_list:
            print(f'Xa = {Xa}')
            ## move to grap position
            movel(Xa, v=VELOCITY, a=ACC)
            time.sleep(0.4)

            ## force control
            task_compliance_ctrl(stx=[1000, 1000, 1000, 100, 100, 100])
            time.sleep(0.4)
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            tool_force = get_tool_force()[2]
            while tool_force < FORCE_THRESHOLD:
                tool_force = get_tool_force()[2]
                time.sleep(0.1)
            release_force()
            release_compliance_ctrl()
            time.sleep(0.7)

            ## if 작은 블럭으로 감지될 때
            current_posx, sol = get_current_posx()
            print(f'current_posx[2] = {current_posx[2]}')

            if current_posx[2] < 49:
                print("Block = Small Block")
                ## block grip
                # z up 10mm
                movel([0, 0, 10, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)
                time.sleep(0.4)
                release()
                # z down 25mm
                movel([0, 0, -25, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)
                time.sleep(0.4)
                grip()
                # z up 90mm
                movel([0, 0, 90, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)

                ## move to release position
                
                if not Xb_check_list[0]:
                # 첫번째 칸 비어있을 때
                    Xb_check_list[0] = True
                    movel(Xb1_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -80, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 작은 블럭같은 경우 z 80 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')
                # 두번째 칸 비어있을 때
                elif not Xb_check_list[1]:
                    Xb_check_list[1] = True
                    movel(Xb2_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -80, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 작은 블럭같은 경우 z 80 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')
                # 세번째 칸 비어있을 때
                elif not Xb_check_list[2]:
                    Xb_check_list[2] = True
                    movel(Xb3_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -80, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 작은 블럭같은 경우 z 80 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')

            ## if 중간 블럭으로 감지될 때
            elif 49 < current_posx[2] < 59:
                print("Block = Midium Block")
                ## block grip
                # z up 10mm
                movel([0, 0, 10, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)
                time.sleep(0.4)
                release()
                # z down 25mm
                movel([0, 0, -25, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)
                time.sleep(0.4)
                grip()
                # z up 90mm
                movel([0, 0, 90, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)

                ## move to release position
                # 첫번째 칸 비어있을 때
                if not Xb_check_list[3]:
                    Xb_check_list[3] = True
                    movel(Xb4_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -70, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 중간 블럭같은 경우 z 70 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')
                # 두번째 칸 비어있을 때
                elif not Xb_check_list[4]:
                    Xb_check_list[4] = True
                    movel(Xb5_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -70, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 중간 블럭같은 경우 z 70 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')
                # 세번째 칸 비어있을 때
                elif not Xb_check_list[5]:
                    Xb_check_list[5] = True
                    movel(Xb6_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -70, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 중간 블럭같은 경우 z 70 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')

            ## if 큰 블럭으로 감지될 때
            elif 59 < current_posx[2]:
                print("Block = Big Block")
                ## block grip
                # z up 10mm
                movel([0, 0, 10, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)
                time.sleep(0.4)
                release()
                # z down 25mm
                movel([0, 0, -25, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)
                time.sleep(0.4)
                grip()
                # z up 90mm
                movel([0, 0, 90, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)

                ## move to release position
                # 첫번째 칸 비어있을 때
                if not Xb_check_list[6]:
                    Xb_check_list[6] = True
                    movel(Xb7_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -60, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 큰 블럭같은 경우 z 60 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')
                # 두번째 칸 비어있을 때
                elif not Xb_check_list[7]:
                    Xb_check_list[7] = True
                    movel(Xb8_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -60, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 큰 블럭같은 경우 z 60 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')
                # 세번째 칸 비어있을 때
                elif not Xb_check_list[8]:
                    Xb_check_list[8] = True
                    movel(Xb9_up, v=VELOCITY, a=ACC)
                    movel([0, 0, -60, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL) ## 큰 블럭같은 경우 z 60 down
                    time.sleep(0.4)
                    print(f'Xb_check_list = {Xb_check_list}')

            ## force control
            task_compliance_ctrl(stx=[1000, 1000, 1000, 100, 100, 100])
            time.sleep(0.4)
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            tool_force = get_tool_force()[2]
            while tool_force < FORCE_THRESHOLD:
                tool_force = get_tool_force()[2]
                time.sleep(0.1)
            release_force()
            release_compliance_ctrl()
            time.sleep(0.4)

            ## release and z up
            release()
            movel([0, 0, 100, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)
            time.sleep(0.4)
            grip()

        # init position
        movej(Jhome, v=VELOCITY, a=ACC)
        print("Finish")

        rclpy.shutdown()

if __name__ == "__main__":
        main()