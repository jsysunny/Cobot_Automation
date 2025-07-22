# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
node = rclpy.create_node("assemble_block_node", namespace=ROBOT_ID)

DR_init.__dsr__node = node
from DSR_ROBOT2 import set_digital_output, get_digital_input, wait

def grip():
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(0.5)

def release():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(0.5)

def main(args=None):
    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            trans,
            set_ref_coord,
            DR_BASE, wait,
            task_compliance_ctrl,
            set_desired_force, DR_FC_MOD_REL,
            check_force_condition, release_force, release_compliance_ctrl, DR_AXIS_Z, mwait,
            DR_TOOL, amove_periodic, move_periodic, amovel
        )
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    set_ref_coord(DR_BASE)

    # 포즈 지정
    pos_home = posj([0, 0, 90, 0, 90, 0])
    block_1_grip = posx(426.39, 215.03 ,45.02, 25.45, -176.82, 24.56 ) # 블록 잡을 위치 지정
    block_2_grip = posx(276.00, 214.17, 46.51, 26.37, -176.08, 24.94) # 블록 잡을 위치 지정
    
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():

        release()
        movej(pos_home, vel = VELOCITY, acc = ACC)
        wait(0.5)

        # ## Pick up
        # movel(block_1_grip, vel = VELOCITY, acc = ACC)
        # wait(0.5)

        # block_1_grip_down = [0, 0, -40, 0, 0, 0]
        # mwait()
        # block_1_grip_up = [0, 0, 100, 0, 0, 0]
        # mwait() 
        
        # movel(block_1_grip_down, vel=VELOCITY, acc = ACC, mod = 1)
        # wait(0.5)
        # grip()
        # wait(0.5)
        # movel(block_1_grip_up, vel = VELOCITY, acc = ACC, mod = 1)

        # ## Assemble_1
        # target1 = posx(485.38,115.14 ,15.38, 12.14, -179.19, 11.50) # 블럭 놓을 위치의 위로 이동
        # movel(target1, vel=VELOCITY, acc=ACC)
        # task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        # time.sleep(1)
        # set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        # time.sleep(1)
        # force_condition = check_force_condition(DR_AXIS_Z, max=20)
        # time.sleep(0.5)
        # while force_condition == 0: # 힘제어로 블럭 놓기
        #     force_condition = check_force_condition(DR_AXIS_Z, max=20)
        #     print(force_condition)
        #     time.sleep(0.5)
    
        # release_force()
        # time.sleep(0.5)
        # release_compliance_ctrl()
        # time.sleep(0.5)
        # release()
        # time.sleep(0.5)
        # target1_up = [0, 0, 100, 0, 0, 0]
        # movel(target1_up, vel=VELOCITY, acc=ACC, mod = 1)

        # movej(pos_home, vel = VELOCITY, acc = ACC)
        # wait(0.5)

        # ## Decomposition_1
        # block_1_pull = posx(483.57, 115.67, 45.82, 2.15, -179.76, 1.99)
        # block_1_pull_down = posx(483.57, 115.67, 5.82, 2.15, -179.76, 1.99)

        # movel(block_1_pull, vel=VELOCITY, acc=ACC)
        # movel(block_1_pull_down, vel=VELOCITY, acc=ACC)
        # wait(0.5)
        # grip()
        # wait(0.5)

        # movel([3, 0, 20, 0, 3.0, 0], vel = 10, acc = 10, mod=DR_FC_MOD_REL)
        # movel([3, 0, 20, 0, -3.0, 0], vel = 10, acc = 10, mod=DR_FC_MOD_REL)
        # wait(0.5)

        # movel(block_1_grip, vel = VELOCITY, acc = ACC)
        # movel(block_1_grip_down, vel=VELOCITY, acc = ACC, mod = 1)
        # wait(0.5)
        # release()
        # wait(0.5)
        # movel(block_1_grip, vel = VELOCITY, acc = ACC)

        # movej(pos_home, vel = VELOCITY, acc = ACC)



        ## Pick up 2
        movel(block_2_grip, vel = VELOCITY, acc = ACC)
        wait(0.5)

        block_2_grip_down = [0, 0, -40, 0, 0, 0]
        mwait()
        block_2_grip_up = [0, 0, 100, 0, 0, 0]
        mwait() 
        
        movel(block_2_grip_down, vel=VELOCITY, acc = ACC, mod = 1)
        wait(0.5)
        grip()
        wait(0.5)
        movel(block_2_grip_up, vel = VELOCITY, acc = ACC, mod = 1)

        ## Assemble_2
        target2 = posx(386.61, 4.03, 29.49, 21.56, -175.37, 19.58) # 블럭 놓을 위치의 위로 이동
        movel(target2, vel=VELOCITY, acc=ACC)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(1)
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(1)
        force_condition = check_force_condition(DR_AXIS_Z, max=20)
        time.sleep(0.5)
        while force_condition == 0: # 힘제어로 블럭 놓기
            force_condition = check_force_condition(DR_AXIS_Z, max=20)
            print(force_condition)
            time.sleep(0.5)

        release_force()
        time.sleep(0.5)
        release_compliance_ctrl()
        time.sleep(0.5)
        release()
        time.sleep(0.5)
        target2_up = [0, 0, 100, 0, 0, 0]
        movel(target2_up, vel=VELOCITY, acc=ACC, mod = 1)

        movej(pos_home, vel = VELOCITY, acc = ACC)
        wait(0.5)

    rclpy.shutdown()

if __name__ == "__main__":
    main()