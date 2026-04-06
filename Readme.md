# Isaac Sim + ROS2 통합 창고(Drone + Mobile Manipulator) 시뮬레이션

> 실행 엔트리: `project/integrated_warehouse_supervisor_v6.py`

이 프로젝트는 **Isaac Sim 5.0** 환경에서
- **드론(crazyflie cf2x)** : 순찰 + 바코드 스캔, 가스 사고 발생 시 **사고 구역으로 긴급 출동**
- **모바일 매니퓰레이터(바퀴+UR-arm+Robotiq 2F-140)** : 사고 위치로 **자동 출동(A*+Reactive)** 후 **밸브 잠금 시퀀스 수행**
- **환경(Action Graph)** : 셔터/밸브/가스 상태를 ROS2 토픽으로 **동기화**

를 하나의 Supervisor 스크립트로 통합합니다.

---

## 1) 실행 환경

- Ubuntu 22.04.x
- Isaac Sim 5.0 (`/isaacsim/` 기준)
- ROS2 Humble (호스트에 설치되어 있어야 하며, Isaac Sim에서 `rclpy`를 로드)

### Python 의존성(Isaac Sim Python에 설치)
`integrated_warehouse_supervisor_v6.py`가 사용하는 외부 패키지:
- `numpy`
- `opencv-python` (`cv2`)
- `Pillow` (map.png 로딩)
- `PyYAML` (map.yaml 로딩)
- `pyzbar` (드론 카메라로 바코드 디코딩)

Isaac Sim 폴더(`/isaacsim`)에서:
```bash
./python.sh -m pip install -U numpy opencv-python Pillow PyYAML pyzbar
```

> `pyzbar`는 시스템에 zbar 라이브러리가 필요할 수 있습니다.
```bash
sudo apt-get update
sudo apt-get install -y libzbar0
```

---

## 2) 파일 배치(중요)

질문에서 주신 **필수 경로 조건**을 그대로 정리했습니다.

### 2.1 `/isaacsim/` 아래에 `project/` 폴더가 있어야 함
- `project/` 폴더에는 최소 아래가 포함되어야 합니다.
  - `project/integrated_warehouse_supervisor_v6.py`
  - `project/map.yaml`, `project/map.png`
  - `project/custom_mobile.usd`
  - `project/custom_mobile/` (파이썬 모듈)
  - `project/barcodes/` (A-1~F-4 PNG 바코드 이미지)

즉 구조 예시:
```
/isaacsim/
  python.sh
  ...
  project/
    integrated_warehouse_supervisor_v6.py
    map.yaml
    map.png
    custom_mobile.usd
    custom_mobile/
      __init__.py
      paths.py
      rig.py
      ...
    barcodes/
      A-1.png ... F-4.png
```

### 2.2 환경 USD 위치
스크립트 기본값은 아래입니다.
- `basic12.usd`는 **`/home/rokey/Downloads/basic12.usd`** 에 있어야 함

다른 경로에 두고 싶다면 실행 시 **`--env-usd`로 경로를 바꿀 수 있습니다.**

### 2.3 `Drone_Group.usd`, `barcodes_final.usd` 위치
- `Drone_Group.usd`
- `barcodes_final.usd`

위 2개 파일은 **`/isaacsim/project/`** 에 있어야 합니다.

> 이유: `basic12.usd`가 내부에서 이 파일들을 **참조(reference/sub-layer)** 하는 형태인 경우가 많고, 스크립트 또한 `stage`에 `/World/Drone_Group/.../cf2x` 프림이 존재한다고 가정합니다.

### 2.4 Robotiq 2F-140 폴더 위치
- `robotiq_2f_140` 폴더는 **바탕화면(Desktop)에 있어야 합니다.**

프로젝트에 포함된 `robotiq_2f_140.zip`를 아래처럼 풀어주세요.
```bash
cd ~/Desktop
unzip /isaacsim/project/robotiq_2f_140.zip
```

> `custom_mobile.usd` 내부에 `robotiq_2f_140` 참조 문자열이 포함되어 있어, 해당 경로에 없으면 USD reference 로딩이 실패할 수 있습니다.

---

## 3) 실행 방법

### 3.1 Supervisor 실행
터미널 1:
```bash
cd /isaacsim
./python.sh project/integrated_warehouse_supervisor_v6.py --map-yaml project/map.yaml
```

옵션 예시:
- 헤드리스:
  ```bash
  ./python.sh project/integrated_warehouse_supervisor_v6.py --headless --map-yaml project/map.yaml
  ```
- `basic12.usd`를 Downloads로 옮기기 싫으면:
  ```bash
  ./python.sh project/integrated_warehouse_supervisor_v6.py --map-yaml project/map.yaml --env-usd /isaacsim/project/basic12.usd
  ```

### 3.2 가스 사고 트리거
터미널 2(ROS2 환경 source 후):
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /main/gas_start std_msgs/msg/String "{data: 'start'}"
```

- **중요:** `integrated_warehouse_supervisor_v6.py`는 `/main/gas_start` 메시지의 `data`를 사용하지 않고, 수신 즉시 **A/B/C/D 중 랜덤 구역을 선택**합니다.

---

## 4) ROS2 토픽 인터페이스

### 4.1 입력(Subscribe)
| 토픽 | 타입 | 의미 |
|---|---|---|
| `/main/gas_start` | `std_msgs/String` | 가스 사고 시작 트리거(내용은 무시, 수신 시 랜덤 구역 선택) |
| `/main/normal` | `std_msgs/Empty` | 상황 종료(수동 종료 트리거) |
| `/main/gas_location` | `std_msgs/String` | 모바일 로봇 출동 목표 좌표: `"x y yaw"` 형식 |
| `/incident/isolation_done_a` | `std_msgs/Bool` | (ActionGraph 등에서) A구역 밸브 잠금 완료 신호 |
| `/incident/isolation_done_b` | `std_msgs/Bool` | 동일 |
| `/incident/isolation_done_c` | `std_msgs/Bool` | 동일 |
| `/incident/isolation_done_d` | `std_msgs/Bool` | 동일 |

수동 종료 예시:
```bash
ros2 topic pub --once /main/normal std_msgs/msg/Empty "{}"
```

### 4.2 출력(Publish)
| 토픽 | 타입 | 의미 |
|---|---|---|
| `/main/gas_location` | `std_msgs/String` | 드론이 사고 구역 도착 후 모바일 출동 좌표 발행 |
| `/main/mobile_arrived` | `std_msgs/Empty` | 모바일이 목표 지점 도착 |
| `/main/mobile_done` | `std_msgs/Empty` | 모바일 작업(밸브 조작 시퀀스) 완료 |
| `/drone/barcode_data` | `std_msgs/String` | 바코드 스캔 결과: `"<slot>:<decoded_text>"` |
| `/drone/gas_ppm` | `std_msgs/String` | 활성 사고 구역 ppm 상태(표시용 문자열) |
| `/hazard/shutter_cmd_1..4` | `std_msgs/Bool` | 셔터 제어 신호(구역별 매핑) |
| `/valve/a_angle_deg` 등 | `std_msgs/Float32` | 밸브 각도(도 단위) |
| `/hazmat/gas_fixed_a_ppm` 등 | `std_msgs/Float32` | 각 구역 가스 농도(ppm) |
| `/hazard/fsm_state_code_a` 등 | `std_msgs/Int32` | 구역별 FSM 상태 코드 |

---

## 5) 시스템 동작 흐름(코드 기준)

### 5.1 큰 흐름
1) **평상시**
- 모바일: `PATROL` (격자 순찰 포인트 사이를 인접 랜덤 이동)
- 드론: `patrol` (정해진 `DRONE_SCAN_LOCATIONS` 경로로 순찰 + 바코드 스캔)

2) **가스 사고 트리거(`/main/gas_start`)**
- 랜덤으로 `A/B/C/D` 중 하나를 사고 구역으로 확정
- `GasSimulator.trigger_leak()`로 해당 구역 ppm 증가
- 드론을 `override` 모드로 전환하여 사고 지점으로 긴급 출동
- 해당 구역 `fsm_state_code`를 `1`로 설정

3) **드론이 사고 지점 도착 → 모바일 호출**
- 드론이 `DISPATCH_POINTS[구역] = (x,y,yaw)`를 만들고
- `/main/gas_location`에 `"x y yaw"` 형식으로 1회 발행

4) **모바일 출동 및 밸브 작업**
- 맵(map.yaml/png)을 이용해 **A* + Reactive**로 출동
- 도착 시:
  - `/main/mobile_arrived` 발행
  - 구역별 셔터를 `True`로 켬
  - `fsm_state_code = 2`
- 매니퓰레이터 시퀀스(스무스 보간):
  - `APPROACH(2s)` → `GRIP_CLOSE(1s)` → `TURN(2s)` → `GRIP_OPEN(1s)` → `HOME(2s)`
  - `TURN` 단계에서 `master_valve_state[구역] = VALVE_CLOSE[구역]`로 밸브 토픽 동기화

5) **Action Graph 완료 신호 대기**
- `/incident/isolation_done_<zone>`가 `True`가 되면 완료 처리
- 15초 이상 지연되면 타임아웃으로 강제 완료 처리
- 완료 시:
  - `/main/mobile_done` 발행
  - `fsm_state_code = 3`
  - `GasSimulator.start_ramp_down()`으로 ppm 감소(기본 8초 동안 0으로)

6) **자동 상황 종료(Auto-Normal)**
- `WAIT_NORMAL` 상태에서 해당 구역 ppm이 0에 도달하면 자동으로 `Normal` 처리
- 또는 `/main/normal`을 수동 발행해도 종료 가능

7) **복귀 및 초기화**
- 모바일: 가장 가까운 순찰 포인트로 복귀 후 `PATROL` 재개
- 드론: `patrol` 재개
- 셔터/밸브/FSM/가스 상태를 초기값으로 복원

---

## 6) 구역별 매핑(코드 상수)

### 6.1 모바일 출동 좌표(드론이 발행하는 `/main/gas_location`)
`DISPATCH_POINTS`:
- A: `(-18.0, 28.1, 1.57)`
- B: `(-3.0, 28.1, 1.57)`
- C: `(-24.2, -5.41, 3.14)`
- D: `(2.96, -5.41, 0.0)`

### 6.2 셔터 제어 매핑
`SHUTTER_MAPPING`:
- A → shutter 1, 4
- B → shutter 1, 2
- C → shutter 3, 4
- D → shutter 2, 3

### 6.3 밸브 각도(도)
`VALVE_OPEN / VALVE_CLOSE`:
- A,B : open=90°, close=0°
- C,D : open=0°,  close=90°

---

## 7) 트러블슈팅

### 7.1 `/World/Drone_Group/.../cf2x` 프림이 없다고 종료됨
스크립트는 아래 프림 경로가 존재한다고 가정합니다.
- `drone_path = "/World/Drone_Group/Drone_Group/Drone_Group/cf2x"`

따라서:
- `basic12.usd`가 정상 로드되었는지
- `Drone_Group.usd`가 참조 경로에 맞게 존재하는지(`/isaacsim/project/Drone_Group.usd`)
- Stage 내 프림 이름이 변경되지 않았는지

를 확인하세요.

### 7.2 바코드 스캔이 안 됨
- 드론 카메라 경로:
  - Left: `.../rsd455/.../Camera_OmniVision_OV9782_Color`
  - Right: `.../rsd455_01/.../Camera_OmniVision_OV9782_Color`
- `pyzbar`/`libzbar0` 설치 여부
- `project/barcodes/` PNG들이 `barcodes_final.usd`에서 참조 가능한 위치인지

### 7.3 `robotiq_2f_140` reference 에러
- `~/Desktop/robotiq_2f_140/` 폴더가 없으면 `custom_mobile.usd` 로딩이 실패할 수 있습니다.

---

## 8) 주요 실행 파라미터(요약)

`--map-yaml` : 필수. `project/map.yaml`

`--env-usd` : 환경 USD 경로(기본 `/home/rokey/Downloads/basic12.usd`)

토픽 이름은 아래 옵션으로 변경 가능:
- `--topic-gas-start` (기본 `/main/gas_start`)
- `--topic-normal` (기본 `/main/normal`)
- `--topic-gas-location` (기본 `/main/gas_location`)
- `--topic-mobile-done` (기본 `/main/mobile_done`)
- `--topic-mobile-arrived` (기본 `/main/mobile_arrived`)

---

## 9) 파일 목록(프로젝트 포함물)

- `integrated_warehouse_supervisor_v6.py` : 메인 Supervisor(ROS2 + Isaac Sim 통합)
- `basic12.usd` : 환경(셔터/밸브/가스/드론 포함 가능)
- `Drone_Group.usd` : 드론 그룹 asset
- `barcodes_final.usd` + `barcodes/*.png` : 바코드 패널/텍스처
- `custom_mobile.usd` + `custom_mobile/*` : 모바일 매니퓰레이터 asset + 제어 유틸
- `map.yaml`, `map.png` : 모바일 출동용 점유지도
- `robotiq_2f_140.zip` : Robotiq gripper asset(Desktop 위치 요구)

