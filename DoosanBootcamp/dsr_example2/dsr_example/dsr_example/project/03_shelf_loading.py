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
        VELOCITY, ACC = 30, 30
        FORCE_THRESHOLD = 15

        Jhome = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 물건 높이값 딕셔너리
        height_dict = {'cup': 92.82, 'block': 131.09, 'small_block': 61.04}
        
        ## 물건 탐색위치 : 높이 150mm로 맞추기
        X_pick_1 = [277.36, 67.84, 150.00, 29.93, -178.19, 29.69]
        X_pick_2 = [425.96, 66.83, 150.00, 41.86, -178.45, 42.18]
        X_pick_3 = [272.11, -80.26, 150.00, 41.48, -178.90, 41.87]

        print('check')
        
        # ==================== A variety of motions ====================

        ## init position
        release()
        grip()
        movej(Jhome, v=VELOCITY, a=ACC)

        ## move to grap position
        movel(X_pick_1, v=VELOCITY, a=ACC)

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

        ## 물건 높이값 측정
        current_posx, sol = get_current_posx()
        print(f'current_posx[2] = {current_posx[2]}')

        ## if 컵으로 감지될 때
        if height_dict['cup']-5 < current_posx[2] < height_dict['cup']-5:
            print("Block = Small Block")
            # z up 10mm
            movel([0, 0, 10, 0, 0, 0], v=VELOCITY, a=ACC, mod=DR_FC_MOD_REL)
            time.sleep(0.4)
            release()








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