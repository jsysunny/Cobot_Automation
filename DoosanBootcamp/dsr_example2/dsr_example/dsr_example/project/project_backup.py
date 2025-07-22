import rclpy
import os
import sys

import time
import numpy as np
from rclpy.executors import MultiThreadedExecutor
import threading

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
    stop,
    DR_HOLD,
    g_node
)

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 10, 10
COLLISION_FORCE_THRESHOLD = 10.0

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

collision_detected = False

def check_collision():
    """
    충돌 감지 함수: 일정한 주기로 툴 포스를 확인하고, 충돌이 감지되면 stop() 호출
    """
    global collision_detected

    while not collision_detected:
        tool_force = get_tool_force()
        force_magnitude = np.linalg.norm(tool_force[:3])  # x, y, z의 합성 벡터 크기 계산

        if force_magnitude > COLLISION_FORCE_THRESHOLD:
            print(f"[Collision Detected] Force: {force_magnitude} N")
            collision_detected = True
            stop(DR_HOLD)
            break

        time.sleep(0.05)  # 50ms 주기로 감지

def move_periodic_thread(amp, period, cnt, ref):
    """
    move_periodic()을 스레드로 실행하여 충돌 시 즉시 멈출 수 있도록 구성
    """
    try:
        move_periodic(amp=amp, period=period, cnt=cnt, ref=ref)
    except Exception as e:
        print(f"[Error in move_periodic] {e}")

def move_with_collision_check(amp, period, cnt, ref):
    """
    move_periodic()과 충돌 감지 스레드를 동시에 실행
    """
    global collision_detected
    collision_detected = False

    # 충돌 감지 스레드 시작
    collision_thread = threading.Thread(target=check_collision)
    collision_thread.start()

    # 이동 스레드 시작
    move_thread = threading.Thread(target=move_periodic_thread, args=(amp, period, cnt, ref))
    move_thread.start()

    # 스레드 대기
    move_thread.join()
    collision_thread.join()

def main(args=None):
    """
    메인 함수: ROS 노드 초기화 및 이동 경로 실행
    """
    rclpy.init(args=args)
    global g_node
    g_node = rclpy.create_node('dsr_control_node', namespace=ROBOT_ID)
#     node = rclpy.create_node('collision_check_node',namespace=ROBOT_ID)
    DR_init.__dsr__node = g_node
   

    # 속도 및 가속도 설정
    set_velx(30, 20)
    set_accx(60, 40)

    # 툴 및 TCP 설정
    set_tool("Tool Weight1")
    set_tcp("GripperDA_v1")

    # 홈 위치로 이동
    Jhome = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
    movej(Jhome, v=VELOCITY, a=ACC)

    # 멀티 스레딩 Executor 생성
    executor = MultiThreadedExecutor()
    executor.add_node(g_node)
        # home = [56.11, 20.33, 72.02, 2.54, 88.79, -85.17]
        # right_down = [42.18, 73.92, 35.88, -1.36, 63.57, -85.17]
        # right_up = [44.98, 74.03, 8.03, -5.05, 68.26, -85.17]
        # left_down = [71.43, 61.93, 57.67, 1.35, 55.24, -85.17]
        # left_up = [80.15, 60.74, 29.14, 1.79, 82.58, -85.17]
    try:
        # 이동 경로 시작
        print("[Move] 오른쪽 이동")
        move_with_collision_check([0.065, 0.0, 0.0, 0.0, 0.0, 0.0], period=2.0, cnt=2, ref=DR_BASE)

        print("[Move] 아래쪽 이동")
        move_with_collision_check([0.0, 0.065, 0.0, 0.0, 0.0, 0.0], period=2.0, cnt=1, ref=DR_BASE)

        print("[Move] 왼쪽 이동")
        move_with_collision_check([-0.065, 0.0, 0.0, 0.0, 0.0, 0.0], period=2.0, cnt=2, ref=DR_BASE)

        print("[Move] 아래쪽 이동")
        move_with_collision_check([0.0, 0.065, 0.0, 0.0, 0.0, 0.0], period=2.0, cnt=1, ref=DR_BASE)

        # ROS 스피닝 (필요 시)
        executor.spin()

    finally:
        # 노드 제거 및 종료
        executor.shutdown()
        g_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
