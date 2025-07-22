# Doosan Robotics Boot Camp(2025.01.06 ~ 2025.07.06)
## 1. ROKEY B-1조 협동-1 Project (ROS2를 활용한 로봇 자동화 공정 시스템 구현 프로젝트) Cobot_Automation
&nbsp;
## 🧠 미라클 모닝: 협동로봇 기반 스마트 출·퇴근 물품 관리 시스템


&nbsp;

## 🔗 출처 및 라이선스

이 프로젝트는 **두산로보틱스(Doosan Robotics Inc.)**에서 배포한 **ROS 2 기반 패키지**를 바탕으로 개발되었습니다.  
소스코드는 [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause)에 따라 공개되어 있으며,  
본 저장소 또한 동일한 라이선스를 따릅니다. 자세한 내용은 `LICENSE` 파일을 참고하세요.

> ⚠️ 해당 저장소는 두산로보틱스의 **공식 저장소가 아니며**,  
> 학습 및 테스트를 위한 **비공식 수정본**을 일부 포함하고 있습니다.  
> 공식 자료는 [두산로보틱스 홈페이지](https://www.doosanrobotics.com/) 및  
> [Doosan GitHub 저장소](https://github.com/DoosanRobotics/doosan-robot2)를 참고해 주세요.

&nbsp;

## 📑 목차

1. [📌 프로젝트 개요](#1--프로젝트-개요)  
2. [🔧 구성 요소](#2--구성-요소)  
3. [💻 사용 기술](#3--사용-기술)  
4. [🧭 동작 흐름 요약](#4--동작-흐름-요약)  
5. [💻 코드 실행 방법](#5--코드-실행-방법)  
6. [📷 시연 영상/이미지](#6--시연-영상--이미지)  
7. [🌟 기대 효과/ 한계점 및 개선점](#7--기대-효과)  
8. [⚠️ 한계점 및 개선점](#8--한계점-및-개선점)
   
&nbsp;
## 1. 📌 프로젝트 개요

출근과 퇴근 시간은 하루 중 반복적인 행동이 몰리는 시간대입니다. 특히 개인 물품을 정리하고 챙기는 과정은 작지만 꾸준한 시간 소모와 불편함을 야기합니다.  
이 프로젝트는 협동로봇(cobot)을 활용하여 **출근 전 물품 꺼내기**와 **퇴근 후 물품 정리/수납**을 자동화하는 시스템입니다.

&nbsp;
### 🎯 기획 의도

- 출근과 퇴근 과정에서 반복적으로 수행되는 **선반 정리 및 수납 작업을 자동화**하고자 했습니다.
- 사용자의 물품 목록을 기반으로, 로봇이 물건을 대신 꺼내주거나 정리해줌으로써 **일상 속 편의성 향상**을 목표로 합니다.
- 이는 단순한 시간 절약을 넘어, **일상 루틴에서의 스트레스 감소**와 **사용자 경험 개선**으로 이어집니다.

&nbsp;
### 🏭 기존 기술의 활용과 협동로봇의 확장 가능성

- 선반 정리 및 물품 수납 작업은 기존에는 주로 **대형 산업용 로봇**을 활용한 자동화 시스템에서 사용되어 왔습니다.
  
    예: **물류창고**에서는 로봇이 선반에서 물건을 꺼내 배송 구역으로 운반하는 자동 피킹 시스템
  
    예: **생산현장**에서는 부품을 정리하거나 공급하기 위해 고정형 로봇팔이 정해진 경로를 따라 반복 작업을 수행

- 이처럼 산업 현장에서의 자동화는 정밀성, 속도, 효율성 측면에서 매우 뛰어나지만,  
  대부분 넓은 공간, 고정된 구조, 고비용 장비를 기반으로 하기 때문에  
  일상 환경에 직접 적용하기에는 한계가 있습니다.

- 본 프로젝트는 이러한 기술을 **협동로봇** 기반으로 소형화하고 유연하게 재구성 하여,  
  좁은 생활 공간에서도 안전하고 직관적으로 사용할 수 있는 시스템으로 구현한 데에 의의가 있습니다.

- 협동로봇은 사람과 함께 근접한 거리에서 작업할 수 있고,  
  별도의 안전 울타리 없이도 충돌 감지, 순응제어 등을 통해  
  사용자 친화적인 작업 환경을 제공합니다.

- 더 나아가, 본 시스템은 **중소형 사업장, 점포, 오피스 환경** 등에서도  
  **물품 정리 및 분류 자동화**에 협동로봇을 적용할 수 있는 가능성을 제시합니다.

- 따라서 본 프로젝트는 기존 산업용 기술의 일상화를 실현함과 동시에,  
  협동로봇을 활용한 차세대 산업 자동화의 새로운 방향성을 함께 보여주는 사례라 할 수 있습니다.

&nbsp;
## 2. 🔧 구성 요소

- **협동로봇 Doosan M0609**: 선반 간 이동 및 물건 그리핑
- **Raspberry Pi4 4gb**: 로봇과 LCD,스피커 간 통신 및 제어를 위한 마이크로컨트롤러
- **LCD**: 현재 동작 상태 및 사용자 입력 정보를 시각적으로 제공
- **스피커**: 음성 안내를 통해 사용자와의 직관적인 인터랙션 제공

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/ed44d43f-d51c-4f3a-9cf1-d87ee4a7d610" /> <img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/b40a74ce-2068-4d40-a3bb-8301a27129e8" />

&nbsp;
## 3. 💻 사용 기술

| 기술 | 내용 |
|------|------|
Ubuntu 22.04 (ROS2 humble)
| 💬 언어 | Python, DSR
 Middleware 통신 ? ROS2|
| 🧠 제어 플랫폼 | ROS2 기반 제어 알고리즘 |
| 🧪 순응제어 | Force 센서를 활용한 충돌 감지 및 위치 파악 |
| 🎮 인터페이스 | 사용자 입력 기반 물품 선택 |
| 🖥 디스플레이/음성 | Raspberry Pi + LCD + 음성 출력 |

&nbsp;
## 4. 🧭 동작 흐름 요약
<img width="600" height="224" alt="image" src="https://github.com/user-attachments/assets/f9b3e453-4a83-46d6-85bd-8abab347f9d9" />
<img width="600" height="1463" alt="image" src="https://github.com/user-attachments/assets/059756fc-f98b-4823-a8c3-b05fa19cbf83" />

&nbsp;
### 🏠 퇴근 모드

0. **사용자 아이템 리스트**  
   - 카드키, 껌, 지갑, 스낵, 텀블러

1. **홈 위치 대기**  
   - Force 센서를 활용해 충돌 감지 상태에서 대기

2. **수납 알고리즘 시작**  
   - y축 방향 외력 감지 (Check Force Condition) → 수납 알고리즘 진입  
   - 💬 음성 출력: `"Good night! Have a sweet dream"`  
   - 📺 LCD 출력: `good night!`

3. **인사 동작 (Good night)**  
   - `Move_periodic` 동작으로 인사 수행  
   - 💬 음성 출력: `"Good night!"`  
   - 📺 LCD 출력: `"Good night!"`

4. **사용자 입력**  
   - 원하는 물체 및 선반 위치 입력  
   - 예: `텀블러 1`

5. **물체 탐색**  
   - ㄹ자 구조로 반복 탐색 수행  
     - Movel 명령으로 x축 400mm, y축 50mm 탐색  
   - 📺 LCD 출력: `"Searching"` Gage 애니메이션 표시

6. **물체 분류 및 Grip 동작**  
   - 비동기 탐색 중 `Get tool force`로 외력 감지 → 물체 존재 확인  
   - 📺 LCD 출력: `"grabbed object!: {Detected name}"`  
   - 💬 음성 출력: `{Detected name}`  
   - 순응제어로 z축 위치 파악 → `height_dict`와 비교하여 분류  
   - `Release` → `Grip` 동작으로 물체 집기

7. **입력 위치에 물품 수납**  
   - **비어있는 경우**: 원래 위치 (`placed_list`)에 수납  
   - **이미 물건이 있는 경우**:  
     - 예: `stacked = [1, 0, 0, 0]` → `stacked = [2, 0, 0, 0]`  
     - x축으로 떨어진 지점에 수납  
   - 📺 LCD 출력: `"Placed object: {Detected name}"`  
   - 💬 음성 출력: `{Detected name}`

8. **그리퍼 홈 위치 복귀**  
   - 수납 완료 후, 그리퍼가 홈 위치로 이동하여 대기  
   - 📺 LCD 출력: `"Request complete"`  
   - 💬 음성 출력: `"Request complete"`

&nbsp;
### 🚪 출근 모드
0. **홈 위치 대기**  
   - Force 센서를 활용해 충돌 감지 상태에서 대기

1. **꺼내기 알고리즘 시작**  
   - x축 방향 외력 감지 (Check Force Condition) → 꺼내기 알고리즘 진입  
   - 💬 음성 출력: `"Have a nice day!"`  
   - 📺 LCD 출력: `"Hello, Have a nice day!"`

2. **인사 동작 (Hello)**  
   - `Move_periodic` 동작으로 인사 수행  
   - 💬 음성 출력: `"Hello!"`  
   - 📺 LCD 출력: `"Hello!"`

3. **사용자 입력**  
   - 원하는 물체 및 선반 위치 입력  
   - 예: `텀블러 1`

4. **물품 위치 비교 및 꺼내기**  
   - **입력값과 위치가 일치하는 경우** → 해당 위치에서 물건 꺼냄  
     - 📺 LCD 출력: `{Detected name} out`  
     - 💬 음성 출력: `{Detected name}`  
   - **불일치하는 경우** → 동작 수행하지 않음

5. **꺼낸 물품 배치**  
   - 최대 5개까지 꺼낼 수 있음  
   - 입력된 순서대로, 홈 위치에서 일정 간격으로 떨어진 위치에 배치

6. **그리퍼 홈 위치 복귀**  
   - 꺼내기 완료 후, 그리퍼가 홈 위치로 복귀  
   - 📺 LCD 출력: `"Request complete"`  
   - 💬 음성 출력: `"Request complete"`

&nbsp;
## 5. 💻 코드 실행 방법

### 🤖 Robot Control Node
- 코드: [`robot_control_node.py`](./rokey_project/robot_control_node.py)

```bash
ros2 run rokey_project robot_control_node
```
### 🍓 Raspberry Pi Node
- 코드: [`feedback_node.py`](./rokcy_project/feedback_node.py)

```bash
ros2 run rokey_project feedback_node
```
&nbsp;
## 6. 📷 시연 영상 / 이미지
> https://youtu.be/bbBvETzXTgY
> <img width="600" height="415" alt="image" src="https://github.com/user-attachments/assets/52fde78e-1d48-4131-9ba5-a52d8baa4287" />
> <img width="600" height="415" alt="image" src="https://github.com/user-attachments/assets/24839a30-8b8b-4170-aeda-596b4a016ea2" />
> <img width="600" height="392" alt="image" src="https://github.com/user-attachments/assets/a3af83a5-ea75-4e62-b817-4eddd1cb01de" />
> <img width="600" height="392" alt="image" src="https://github.com/user-attachments/assets/bfb44ccb-1988-4563-abb9-64399405e04d" />


&nbsp;
## 7. 🌟 기대 효과

- 일상생활에 협동로봇 도입 가능성 증진
- 출퇴근 시간의 불편함 해소
- 다양한 물품 및 센서로의 확장성 기대

## ⚠️ 한계점 및 개선점

- 그리퍼의 중량, 가속도/속도에 따른 force 조절 미흡 → 물체 밀림 발생
- 선반 높이 제한 → 워크스페이스 조정 필요

&nbsp;
## 🙌 팀원

- 백홍하,정서윤,정민섭,서형원

## 📁 폴더 구조 (예시)

