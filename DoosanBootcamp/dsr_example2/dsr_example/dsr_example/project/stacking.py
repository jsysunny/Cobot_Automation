import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("rokey_stacking", namespace=ROBOT_ID)
DR_init.__dsr__node = node

ON, OFF = 1, 0
HOME_READY = [0, 0, 90, 0, 90, 0]

import time

try:
    from DSR_ROBOT2 import (
        get_digital_input,
        set_digital_output,
        get_current_posx,
        trans,
        set_tool,
        set_tcp,
        movej,
        movel,
        wait,
        mwait,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,
        check_force_condition,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")

def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)
        print("Wait for digital input")
        pass

def release():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait_digital_input(2)

def grip():
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait_digital_input(1)


class Box:
    def __init__(self, id, pos_id, position):
        self.id = id
        self.pos_id = pos_id
        self.position = position
        self.target_offset = 100
        self.stacked = False

    def set_pos_id(self, pos_id):
        self.pos_id = pos_id

    def set_box_id(self, id):
        self.id = id
    
    def set_position(self, pos_list):
        self.position = pos_list
    
    def info(self):
        return f"id : {self.id}\nposition : {self.pos_id} -> {self.position}\n=====\n"

    def __move_to_pos(self, target_pos, action=None):
        init_pos = get_current_posx()[0]
        print("init_pos:", init_pos)
        mwait()
        time.sleep(0.1)

        ready_pos = list(init_pos)
        ready_pos[2] = 350
        print("ready_pos:", ready_pos)

        print("moving to ready_pos")
        movel(ready_pos, vel=VELOCITY, acc=ACC, mod=0)
        mwait()
        time.sleep(0.1)

        print("moving to target_pos")
        movel(target_pos, vel=VELOCITY, acc=ACC, mod=0)
        print("target_pos:", target_pos)
        mwait()
        time.sleep(0.1)

        if action == 'grip':
            grip()
        elif action == 'release':
            release()
        mwait()

        movel(ready_pos, vel=VELOCITY, acc=ACC, mod=0)
        movej(HOME_READY, vel=VELOCITY, acc=ACC)
        return target_pos


    def stack(self):
        if self.stacked:
            print(f"Box {self.id} is already stacked!")
            return
        if not self.position:
            print("No position set for stacking.")
            return
        self.__move_to_pos(self.position, action='release')
        self.stacked=True

    def unstack(self):
        if not self.stacked:
            print(f"Box {self.id} is already unstacked!")
            return
        if not self.position:
            print("No position set for stacking.")
            return
        self.__move_to_pos(self.position, action='grip  ')
        self.stacked = False

def to_grip():
    movej(HOME_READY, vel=VELOCITY, acc=ACC)
    grip()
    mwait()
    movel([0,0,-100,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    time.sleep(1)
    set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    while not check_force_condition(DR_AXIS_Z, max=5):
        pass
    print("end force control")
    release_force()
    release_compliance_ctrl()
    time.sleep(1)
    movel([0,0,20,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    release()
    movel([0,0,-35,0,0,0], vel=VELOCITY, acc=ACC, mod=1)
    grip()
    movej(HOME_READY, vel=VELOCITY, acc=ACC)


def main():
    box_dict = {}
    while rclpy.ok():
    	""" Your Code Here """
    rclpy.shutdown()

if __name__ == "__main__":
    main()
