import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
node = rclpy.create_node("assemble_block_node", namespace=ROBOT_ID)
DR_init.__dsr__node = node

## DSR 언어, 인자 import
try: 
    from DSR_ROBOT2 import (
        set_tool,
        set_tcp,
        movej,
        movel,
        set_ref_coord,
        DR_BASE,
        DR_TOOL,
        get_tool_force,
        get_current_posx,
        DR_AXIS_Z,
        DR_AXIS_Y,
        DR_AXIS_X,
        amovel,
        amovej,
        task_compliance_ctrl,
        set_desired_force,
        DR_FC_MOD_REL,
        check_force_condition,
        release_force,
        release_compliance_ctrl,
        set_digital_output,
        get_digital_input,
        mwait,
        fkin,
        move_periodic,
    )
    from DR_common2 import posx, posj
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

## 그리퍼 setting
set_tool("Tool Weight1")
set_tcp("GripperDA_v1")

'''그리퍼 조작 함수'''
def grip():
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    time.sleep(1.5)

def release():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(1.5)

'''물체명, 적재 위치 지정하는 함수'''
def get_user_inputs():
    print("물체명과 위치 번호를 입력하세요 (예: '텀블러 1').\n엔터만 입력하면 종료됩니다.\n")
    inputs = []
    while True:
        line = input("입력: ").strip()
        if line == "":
            break
        inputs.append(line)
    return inputs

'''물체 높이로 물체를 판별하는 함수'''
def identify_object_by_z(pos, height_dict, tol=5):
    for name, ref_z in height_dict.items():
        if abs(pos - ref_z) <= tol:
            return name
    return None


def main(args=None):

    set_ref_coord(DR_BASE)
    
    # ==================== Set values ====================
    VELOCITY, ACC = 30, 30
    THRESHOLD_FORCE = 2
    
    ## 경유점 pose
    J_grip_home = [-79.50, 7.48, 71.07, 0.01, 101.23, -79.72]
    J_grip_right = [-101.82, 4.64, 74.69, -0.21, 100.05, -101.71]
    J_grip_left = [-57.26, 14.96, 62.37, -0.63, 102.34, -57.25]

    ## place pose
    X_left_up_place = [187.11, -599.38, 159.21, 44.76, -178.08, 46.36]
    X_left_down_place = [193.88, -429.41, -22.41, 36.01, -178.39, 37.11]
    X_right_up_place = [-162.82, -605.87, 151.53, 7.36, -178.37, 8.57]
    X_right_down_place = [-148.12, -421.63, -33.47, 158.84, 178.27, 160.59]

    ## place pose list
    place_list = [X_left_up_place, X_left_down_place, X_right_up_place, X_right_down_place]

    # ==================== Motions ====================

    while rclpy.ok(): # 구동부
        ## initial position
        print("move")
        
        # movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        # movej([45.0, 0, 90, 0, -90, 0], vel=VELOCITY, acc=ACC)
        # release()
        # move_periodic(amp =[0, 0, 60, 0, 7, 0], period=[0, 0, 7, 0, 1.5, 0], atime=0.5, repeat=2, ref=DR_TOOL)
        # grip()
        # movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)



        out_place_list_x = [[277.25, 216.84], [427.80, 216.62], [276.85, 68.43], [426.63, 66.08], [425.55, -83.28]]
        movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
        time.sleep(0.5)
        current_posx = get_current_posx()[0]
        time.sleep(0.5)

        for i in range(len(out_place_list_x)):
            current_posx[0] = out_place_list_x[i][0]
            current_posx[1] = out_place_list_x[i][1]
            time.sleep(0.3)
            movel(current_posx, vel=VELOCITY, acc=ACC)
            time.sleep(0.5)

        time.sleep(5)

    rclpy.shutdown()
if __name__ == "__main__":
    main()