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
    time.sleep(1)

def release():
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    time.sleep(1)

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
    # 포즈 지정
    # current_pose = start_pose
    # delta_list = [
    #     [0, 450, 0, 0, 0, 0],
    #     [-10, 0, 0, 0, 0, 0],
    #     [0, -450, 0, 0, 0, 0]
    # ]
    
    # ==================== Set values ====================
    VELOCITY, ACC = 60, 60
    THRESHOLD_FORCE = 2
    
    ## pose
    J_right_upper = [-79.50,7.48,85.07,0.01,101.23,-79.72]
    J_grip_home = [-79.50, 7.48, 71.07, 0.01, 101.23, -79.72]
    J_grip_right = [-101.82, 4.64, 74.69, -0.21, 100.05, -101.71]
    J_grip_left = [-57.26, 14.96, 62.37, -0.63, 102.34, -57.25]

    J_right_up_place = [-105.33, 40.85, 36.37, -0.55, 102.66, -105.12]
    J_left_up_place = [-73.14, 39.54, 40.98, -0.71, 99.08, -72.76]

    place_list = [J_left_up_place,
                  [-112.82,25.04,102.76,-0.15,52.24,-112.60],
                  J_right_up_place,
                  [-75.05,42.44,41.49,1.08,99.17,-74.33]]

    # place_list = [[-107.50,41.73,42.56,-0.16,99.45,-107.83],
    #               [-112.82,25.04,102.76,-0.15,52.24,-112.60],
    #               [-68.84,26.89,100.16,1.37,55.58,-69.53],
    #               [-75.05,42.44,41.49,1.08,99.17,-74.33]]

    #inputs = get_user_inputs()

    # ==================== Motions ====================

    while rclpy.ok(): # 구동부
        ## initial position
        print("move")
        movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
        release()
        grip()

        ## 로봇 쳤을 때 동작 시작
        put_cnt=0
        out_cnt=0
        if put_cnt==0 or out_cnt==0:
            put_true=False
            out_true=False
            val_y = check_force_condition(DR_AXIS_Y, 10, 30, DR_TOOL)
            val_x = check_force_condition(DR_AXIS_X, 10, 30, DR_TOOL)
            print(f"check_force_condition Y: {val_y}, X: {val_x}")
            while True:
                val_y = check_force_condition(DR_AXIS_Y, 10, 30, DR_TOOL)
                val_x = check_force_condition(DR_AXIS_X, 10, 30, DR_TOOL)

                ## y 방향으로 쳤을 때 꺼내기 시작
                if val_y == 0:
                    print(f"check_force_condition Y: {val_y}, X: {val_x}")
                    print("수납을 시작합니다.")
                    inputs = get_user_inputs()
                    print(f'inputs = {inputs}')
                    put_true=True
                    put_cnt+=1
                    break

                ## x 방향으로 쳤을 때 꺼내기 시작
                elif val_x == 0:
                    print(f"check_force_condition Y: {val_y}, X: {val_x}")
                    print("꺼내기를 시작합니다.")
                    out_true=True
                    out_cnt+=1
                    break

        while put_true:
            ## ㄹ자 탐색
            movel([162.30, 305.92, 100.00, 53.48, -179.14, 53.20], vel=VELOCITY, acc=ACC) # 탐색 초기 위치
            cnt=0
            i=1
            force_triggered=False
            #duration= 400/60=6.7
            while cnt<9 and not force_triggered:
                movel([0, -50, 0, 0, 0, 0], vel=60, acc=60,mod=1)
                if i%2 != 0:
                    amovel([400, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)
                    i+=1
                else:
                    amovel([-400, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)
                    i+=1
                start_time = time.time()
                while time.time() - start_time < 6.7:
                    fx, fy, fz = get_tool_force()[:3]
                    total_force = (fx**2 + fy**2 + fz**2)**0.5
                    print(f'total_force: {total_force:.2f} N')
                    threshold_force = 15
                    if total_force >= threshold_force:
                        force_triggered=True
                        new_pos = get_current_posx()[0]
                        print(f'Force reached: {total_force:.2f} N')
                        break
                    time.sleep(0.1)
                cnt+=1

            if force_triggered:
                #new_pos=new_pos+[-20,0,0,0,0,0]
                movel(new_pos,vel=60,acc=60,mod=0)
                movel([0, 0, 120, 0, 0, 0], vel=60, acc=60,mod=1)
                if i%2!=0:
                    movel([-100,0,0,0,0,0],vel=60,acc=60,mod=1)
                else:
                    movel([+100,0,0,0,0,0],vel=60,acc=60,mod=1)
                #movel([0, 0, -20, 0, 0, 0], vel=60, acc=60,mod=1)

                ## force control
                task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                time.sleep(1)
                set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                time.sleep(1)

                # 높이 감지
                force_condition = check_force_condition(DR_AXIS_Z, max=20)
                time.sleep(0.5)
                while force_condition == 0:
                    force_condition = check_force_condition(DR_AXIS_Z, max=20)
                    print(force_condition)
                    time.sleep(0.5)
                pos = get_current_posx()[0][2]
                print(f'pos: {pos:.2f} m')
                release_force()
                time.sleep(1)
                release_compliance_ctrl()
                time.sleep(1)
                movel([0, 0, 50, 0, 0, 0], vel=60, acc=60,mod=1)
                release()
                movel([0, 0, -100, 0, 0, 0], vel=60, acc=60,mod=1)
                grip()
                movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

                place_dict = {}
                height_dict = {'텀블러': 197,'지갑': 30,'버즈': 25,'우산': 50,'양치세트': 65,'카드키': 6,'폰': 13,'안경집': 32}
                detected_name = identify_object_by_z(pos, height_dict)
                print(f'감징된 물체 = {detected_name}')
                
                if detected_name:
                    for item in inputs:
                        print(f'item = {item}')
                        name, index = item.split()
                        index = int(index)
                        if name == detected_name:
                            print(f"감지된 물체 '{detected_name}' → 사용자 입력 '{name} {index}'에 따라 이동합니다.")

                            ## move
                            movej(J_grip_home, vel=30, acc=30, mod=0)

                            if index >= 3: 
                                movej(J_grip_right, vel=30, acc=30, mod=0)
                            elif index < 3:
                                movej(J_grip_left, vel=30, acc=30, mod=0)

                            movej(place_list[index-1],vel=30, acc=30,mod=0)

                            ## force control
                            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                            time.sleep(1)
                            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                            time.sleep(1)
                            force_condition = check_force_condition(DR_AXIS_Z, max=20)
                            time.sleep(0.5)
                            while force_condition == 0:
                                force_condition = check_force_condition(DR_AXIS_Z, max=20)
                                time.sleep(0.5)
                            release_force()
                            time.sleep(1)
                            release_compliance_ctrl()
                            time.sleep(1)
                            release()
                            movel([0, 0, 100, 0, 0, 0], vel=60, acc=60,mod=1)

                            ## home으로 이동
                            print("release후 홈으로 이동합니다")
                            
                            if index >= 3: 
                                movej(J_grip_right, vel=30, acc=30, mod=0)
                            elif index < 3:
                                movej(J_grip_left, vel=30, acc=30, mod=0)

                            movej(J_grip_home, vel=30, acc=30, mod=0)
                            movej([0,0,90,0,90,0],vel=30, acc=30,mod=0)

                            ## place_dict에 저장
                            place_dict[detected_name] = index
                            print("place_dict에 저장합니다")
                            break
                    else:
                        print(f" 감지된 물체 '{detected_name}'는 사용자 입력에 포함되어 있지 않습니다.")
                else:
                    print("z값으로부터 인식된 물체가 없습니다.")
            grip()

        if out_true:
            if not place_dict:
                print(" 저장된 물체가 없습니다. 먼저 물체를 감지하고 위치를 저장해주세요.")
            else:
                for item in place_dict.items():
                    print(f"'{name}' 를 저장된 위치 {index}에서 가져옵니다")
                    name, index = item.split()
                    index = int(index)
                    release()
                    movej(place_list[index-1],vel=60, acc=60,mod=0)
                    grip()
                    movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
                    movel([0,0,-50,0,0,0],vel=VELOCITY, acc=ACC,mod=1)
                    release()
                    time.sleep(1)
    rclpy.shutdown()
if __name__ == "__main__":
    main()