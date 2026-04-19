# Sensor Sync

Teensy 4.1 기반 RGB synchronizer + ROS2 제어 노드입니다.

## 개요

이 패키지는 GPS PPS를 기준으로 Stereo RGB 카메라 트리거를 생성합니다.
정식 운영 기본 모드는 `gps_pps`이고, GPS 없이도 디버깅할 수 있도록
`internal_timer` 모드를 유지합니다.

```text
┌─────────────────────────────────────────────────────────────┐
│                      Teensy 4.1                             │
│                                                             │
│   Pin 2  ──→  FLIR BFS Stereo RGB x2 trigger output         │
│   Pin 3  ←──  GPS PPS 1 Hz input                            │
│   Pin 10 ←──  External trigger input (legacy debug)         │
│   Pin 11 ──→  Oscilloscope monitor (same as Pin 2)          │
│   Pin 13 ──→  Status LED                                    │
│                                                             │
│   Event / LiDAR use the GPS PPS direct path separately      │
└─────────────────────────────────────────────────────────────┘
```

## 디렉토리 구조

```text
sensor_sync/
├── firmware/
│   └── sync.ino
├── sensor_sync/
│   ├── protocol.py
│   └── sync_node.py
├── test/
│   └── test_protocol.py
├── package.xml
├── setup.py
└── README.md
```

## 펌웨어 기본 동작

- 기본 모드: `gps_pps`
- 기본 입력: Pin 3 GPS PPS rising edge
- fallback 모드: `internal_timer`
- 추가 fallback: `external_trigger` (legacy debug)
- ROS 연결 시 mode, FPS, start/stop을 제어할 수 있습니다.

## ROS2 노드 빌드

### 요구사항
- ROS2 Humble
- Python 패키지: `pyserial`

```bash
pip install pyserial
```

### 빌드

```bash
cd /root/ros2_ws
colcon build --packages-select sensor_sync
source install/setup.bash
```

## 실행

### 기본 실행

```bash
ros2 run sensor_sync sync_node
```

기본값:
- `sync_mode:=gps_pps`
- `fps:=1.0`
- `auto_connect:=true`
- 현재 기본 펄스폭(`10 ms`) 기준 보수적 지원 범위: `1-60 Hz`

### RGB FPS 지정

```bash
ros2 run sensor_sync sync_node --ros-args -p fps:=30.0
```

### internal timer 디버깅 모드

```bash
ros2 run sensor_sync sync_node --ros-args -p sync_mode:=internal_timer -p fps:=10.0
```

### 시리얼 포트 지정

```bash
ros2 run sensor_sync sync_node --ros-args -p port:=/dev/ttyACM0 -p fps:=30.0
```

### 모든 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `port` | `/dev/ttyACM0` | Teensy 시리얼 포트 |
| `baud` | `115200` | 시리얼 통신 속도 |
| `fps` | `1.0` | Stereo RGB target FPS (1-60, current conservative limit) |
| `sync_mode` | `gps_pps` | `gps_pps`, `internal_timer`, `external_trigger` |
| `auto_connect` | `true` | 시작 시 자동 연결 |

## ROS2 인터페이스

### 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `~/start` | `std_srvs/Trigger` | 동기화 시작 |
| `~/stop` | `std_srvs/Trigger` | 동기화 중지 |
| `~/trigger` | `std_srvs/Trigger` | 단일 RGB 트리거 발생 |
| `~/connect` | `std_srvs/Trigger` | Teensy 연결 |
| `~/disconnect` | `std_srvs/Trigger` | Teensy 연결 해제 |

### 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `~/status` | `std_msgs/String` | 연결/동작/모드/PPS 상태 요약 |
| `~/current_fps` | `std_msgs/Float32` | 현재 적용된 FPS |
| `~/active_mode` | `std_msgs/String` | 현재 활성 모드 |
| `~/pps_locked` | `std_msgs/Bool` | GPS PPS lock 상태 |
| `~/pps_age_ms` | `std_msgs/Int32` | 마지막 PPS 이후 경과 시간 |

### 사용 예시

```bash
# 동기화 시작
ros2 service call /sync_controller/start std_srvs/srv/Trigger

# 동기화 중지
ros2 service call /sync_controller/stop std_srvs/srv/Trigger

# FPS 변경
ros2 param set /sync_controller fps 60.0

# GPS PPS -> internal timer 모드 전환
ros2 param set /sync_controller sync_mode internal_timer

# 상태 확인
ros2 topic echo /sync_controller/status
ros2 topic echo /sync_controller/pps_locked
```

## 동작 원리

### 기본 운영 모드: GPS PPS

```text
GPS PPS 1 Hz (Pin 3 input)
        │
        └──→ Teensy가 초 경계를 기준으로 잡음
              │
              └──→ Pin 2 RGB trigger Hz 생성
```

- RGB trigger는 GPS 초 경계에 정렬되도록 생성됩니다.
- ROS에서는 `pps_locked`, `pps_age_ms`로 상태를 볼 수 있습니다.
- 현재 `TRIGGER_PULSE_US=10000` 기준으로는 `60 Hz`를 보수적 상한으로 둡니다.
- FLIR 전기적 검증 후 pulse width를 줄이면 상한은 다시 올릴 수 있습니다.

### 디버그 모드: internal timer

- GPS 없이도 RGB trigger를 단독 테스트할 수 있습니다.
- 카메라 bring-up이나 배선 디버깅에 사용합니다.
- 정식 acquisition 기본 모드는 아닙니다.

## 하드웨어 연결

| Teensy Pin | 연결 대상 | 신호 방향 | 비고 |
|------------|----------|----------|------|
| Pin 2 | FLIR BFS GPIO (level shifter 경유) | OUTPUT | RGB trigger |
| Pin 3 | GPS PPS | INPUT | 기본 시간 기준 |
| Pin 10 | 외부 트리거 소스 | INPUT | legacy debug |
| Pin 11 | 오실로스코프 | OUTPUT | RGB trigger monitor |
| Pin 13 | 온보드 LED | OUTPUT | 상태 표시 |
| GND | 공통 그라운드 | - | 필수 |

### 주의사항

- FLIR 카메라는 5V trigger가 필요할 수 있으므로 레벨 시프터를 사용합니다.
- GPS PPS와 Teensy 입력 전압 레벨 호환 여부를 확인합니다.
- 모든 장치의 GND를 공통으로 연결합니다.

## 시리얼 명령어

Teensy에 직접 시리얼 명령어를 보낼 수 있습니다.

| 명령어 | 단축키 | 설명 |
|--------|--------|------|
| `START` | `S` | 동기화 시작 |
| `STOP` | `X` | 동기화 중지 |
| `TRIGGER` | `T` | 단일 RGB 트리거 |
| `FPS n` | `F n` | FPS 설정 |
| `MODE GPS_PPS` | `G` | GPS PPS 모드 |
| `MODE INTERNAL` | `I` | 내부 타이머 모드 |
| `MODE EXTERNAL` | `E` | 외부 트리거 모드 |
| `STATUS` | `?` | 상태 출력 |
| `RESET` | `R` | 프레임/카운터 리셋 |
| `HELP` | `H` | 도움말 |

```bash
screen /dev/ttyACM0 115200
```

## 테스트

### 프로토콜 단위 테스트

```bash
cd /root/ros2_ws/src/sensor_sync
PYTHONPATH=. python3 -m unittest discover -s test -p 'test_*.py'
```

### GPS PPS lock 디버깅

```bash
ros2 topic echo /sync_controller/pps_locked
ros2 topic echo /sync_controller/pps_age_ms
```

Pin 3 입력 배선, GND 공통, PPS 전압 레벨을 먼저 확인합니다.
