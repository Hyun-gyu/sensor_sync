# Sensor Sync

Teensy 4.1 기반 다중 센서 동기화 시스템 + ROS2 제어 노드

## 개요

여러 센서(스테레오 카메라, 이벤트 카메라, LiDAR)를 마이크로초 단위로 정밀하게 동기화하는 하드웨어 트리거 시스템입니다.

```
┌─────────────────────────────────────────────────────────────┐
│                      Teensy 4.1                             │
│                                                             │
│   Pin 2  ──→  FLIR BFS 스테레오 카메라 x2  (30Hz, 가변)     │
│   Pin 3  ──→  DAVIS 360 이벤트 카메라      (1Hz)            │
│   Pin 4  ──→  Velodyne Ultra Puck LiDAR   (1Hz)            │
│   Pin 11 ──→  오실로스코프 모니터링        (Pin 2 동일)     │
│   Pin 13 ──→  상태 LED                                      │
│                                                             │
│   Pin 10 ←──  외부 트리거 입력 (선택)                       │
└─────────────────────────────────────────────────────────────┘
```

## 디렉토리 구조

```
sensor_sync/
├── firmware/
│   └── sync.ino           # Teensy 펌웨어
├── sensor_sync/
│   └── sync_node.py       # ROS2 제어 노드
├── package.xml
├── setup.py
└── README.md
```

---

## 1. Teensy 펌웨어 설치

### 요구사항
- [Arduino IDE](https://www.arduino.cc/en/software) 또는 [PlatformIO](https://platformio.org/)
- [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) 애드온

### 업로드
1. Arduino IDE에서 `firmware/sync.ino` 열기
2. Tools → Board → Teensy 4.1 선택
3. Tools → USB Type → Serial 선택
4. Upload 버튼 클릭

### 펌웨어 기본 동작
- **전원 연결 시**: 1Hz로 자동 시작 (ROS 미연결 시에도 동작)
- **ROS 연결 시**: ROS 노드에서 FPS 및 start/stop 제어 가능

---

## 2. ROS2 노드 빌드

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

---

## 3. 실행

### 기본 실행 (1Hz, 자동 연결)
```bash
ros2 run sensor_sync sync_node
```

### FPS 지정
```bash
ros2 run sensor_sync sync_node --ros-args -p fps:=30.0
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
| `fps` | `1.0` | 스테레오 카메라 FPS (1-120) |
| `auto_connect` | `true` | 시작 시 자동 연결 |

---

## 4. ROS2 인터페이스

### 서비스
| 서비스 | 타입 | 설명 |
|--------|------|------|
| `~/start` | `std_srvs/Trigger` | 동기화 시작 |
| `~/stop` | `std_srvs/Trigger` | 동기화 중지 |
| `~/trigger` | `std_srvs/Trigger` | 단일 트리거 발생 |
| `~/connect` | `std_srvs/Trigger` | Teensy 연결 |
| `~/disconnect` | `std_srvs/Trigger` | Teensy 연결 해제 |

### 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `~/status` | `std_msgs/String` | 연결 및 동작 상태 |
| `~/current_fps` | `std_msgs/Float32` | 현재 설정된 FPS |

### 사용 예시
```bash
# 동기화 시작
ros2 service call /sync_controller/start std_srvs/srv/Trigger

# 동기화 중지
ros2 service call /sync_controller/stop std_srvs/srv/Trigger

# FPS 변경 (실행 중에도 가능)
ros2 param set /sync_controller fps 60.0

# 상태 확인
ros2 topic echo /sync_controller/status
```

---

## 5. 동작 원리

### 단일 마스터 타이머 동기화
```
마스터 타이머 (Stereo FPS 기준)
        │
        ├──→ 매 프레임: Pin 2 트리거 (스테레오 카메라)
        │
        └──→ 매 N프레임: Pin 3, 4 동시 트리거 (DAVIS + Velodyne)
             (N = Stereo FPS / 1Hz)
```

- **Drift 없음**: 모든 트리거가 단일 타이머에서 파생
- **정확한 동기화**: 1Hz 센서 트리거는 항상 스테레오 프레임과 일치

### 예시: 30Hz 설정
| 센서 | 핀 | 주파수 | 트리거 타이밍 |
|------|-----|--------|--------------|
| 스테레오 카메라 | Pin 2 | 30Hz | 매 프레임 |
| DAVIS 360 | Pin 3 | 1Hz | 매 30번째 프레임 |
| Velodyne | Pin 4 | 1Hz | 매 30번째 프레임 (Pin 3과 동시) |

### 트리거 신호 특성
- **전압**: 3.3V (Teensy 4.1 출력)
- **펄스 폭**: 10ms (설정 가능)
- **엣지**: Rising edge 트리거

---

## 6. 하드웨어 연결

### 핀 배치
| Teensy Pin | 연결 대상 | 신호 방향 | 비고 |
|------------|----------|----------|------|
| Pin 2 | FLIR BFS GPIO (트랜지스터 경유) | OUTPUT | 30Hz 트리거 |
| Pin 3 | DAVIS 360 SIGNAL_IN | OUTPUT | 1Hz 트리거 |
| Pin 4 | Velodyne SYNC_IN | OUTPUT | 1Hz 트리거 |
| Pin 10 | 외부 트리거 소스 | INPUT | 선택사항 |
| Pin 11 | 오실로스코프 | OUTPUT | 디버깅용 |
| Pin 13 | 온보드 LED | OUTPUT | 상태 표시 |
| GND | 공통 그라운드 | - | 필수 |

### 주의사항
- FLIR 카메라는 5V 트리거가 필요할 수 있음 → 트랜지스터 레벨 시프터 사용
- 모든 장치의 GND를 공통으로 연결

---

## 7. 시리얼 명령어 (디버깅용)

Teensy에 직접 시리얼 명령어를 보낼 수 있습니다:

| 명령어 | 단축키 | 설명 |
|--------|--------|------|
| `START` | `S` | 동기화 시작 |
| `STOP` | `X` | 동기화 중지 |
| `TRIGGER` | `T` | 단일 트리거 |
| `FPS n` | `F n` | FPS 설정 (예: `FPS 30`) |
| `STATUS` | `?` | 상태 출력 |
| `HELP` | `H` | 도움말 |
| `RESET` | `R` | 프레임 카운터 리셋 |
| `INTERNAL` | `I` | 내부 타이머 모드 |
| `EXTERNAL` | `E` | 외부 트리거 모드 |

```bash
# 시리얼 모니터 연결 (115200 baud)
screen /dev/ttyACM0 115200
# 또는
minicom -D /dev/ttyACM0 -b 115200
```

---

## 8. 트러블슈팅

### Teensy가 인식되지 않음
```bash
# 연결된 장치 확인
ls /dev/ttyACM* /dev/ttyUSB*

# 권한 문제 시
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인
```

### 오실로스코프에서 신호가 안 보임
1. **연결 확인**: GND → Teensy GND, 프로브 → Pin 2 (5V 아님!)
2. **트리거 설정**: Auto 모드, Rising edge, 1.5V 레벨
3. **시간축**: 100ms/div 이상 (1Hz 신호 확인용)

### ROS 노드가 연결 실패
```bash
# Teensy 포트 확인
ls -la /dev/ttyACM*

# 다른 프로세스가 포트 점유 중인지 확인
fuser /dev/ttyACM0
```
